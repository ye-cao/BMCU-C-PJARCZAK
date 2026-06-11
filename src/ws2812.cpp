/**
 * @file ws2812.cpp
 * @brief WS2812B RGB LED 驱动实现
 *
 * 使用位操作 (bit-bang) 方式在 GPIO 引脚上产生 WS2812B 协议时序。
 * 通过 SysTick (STK) 定时器进行精确计时, 在关中断状态下逐位发送。
 *
 * 时序参数 (基于 HCLK=144MHz, STK=HCLK/8=18MHz):
 *   - 1 tick = 55.56ns
 *   - T0H = 7 ticks = 0.389us (逻辑0高电平时间)
 *   - T1H = 15 ticks = 0.833us (逻辑1高电平时间)
 *   - 位周期 = 22 ticks = 1.222us
 *   - 复位 = 100us ( datasheet 要求 ≥50us)
 *
 * 颜色格式: GRB (Green-Red-Blue), MSB 优先
 */

#include "ws2812.h"
#include "hal/time_hw.h"
#include "hal/irq_wch.h"

// WS2812B 时序参数 (数据手册):
//  - TH+TL = 1.25us ±600ns
//  - T0H   = 0.4us  ±150ns
//  - T1H   = 0.85us ±150ns
//  - RES low >= 50us
//
// STK = HCLK/8. 当 HCLK=144MHz => STK=18MHz => 55.56ns/tick.

/** @brief 单个数据位的总周期: 22 ticks = 1.222us @18MHz */
#define WS2812_TBIT_TICKS  (22u)
/** @brief 逻辑0高电平时间: 7 ticks = 0.389us @18MHz */
#define WS2812_T0H_TICKS   (7u)
/** @brief 逻辑1高电平时间: 15 ticks = 0.833us @18MHz */
#define WS2812_T1H_TICKS   (15u)

/** @brief 复位脉冲时间 (ticks), 在 init() 中根据 STK 频率计算 */
static uint32_t g_ws2812_rst_ticks = 0;

/**
 * @brief GPIO 置位宏: 将指定引脚设为高电平
 * @param p GPIO 端口指针
 * @param m 引脚掩码
 */
#define RGB_H(p, m) do{ (p)->BSHR = (uint32_t)(m); }while(0)
/**
 * @brief GPIO 清零宏: 将指定引脚设为低电平
 * @param p GPIO 端口指针
 * @param m 引脚掩码
 */
#define RGB_L(p, m) do{ (p)->BCR  = (uint32_t)(m); }while(0)

/**
 * @brief 根据 GPIO 端口指针使能对应的外设时钟
 *
 * CH32V203 每个 GPIO 端口有独立的时钟使能位。
 * 在配置 GPIO 前必须先使能时钟。
 *
 * @param p GPIO 端口指针 (GPIOA/B/C/D)
 */
static inline void enable_gpio_clock(GPIO_TypeDef* p)
{
    if      (p == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (p == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (p == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (p == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

/**
 * @brief 忙等待直到 SysTick 计数器到达指定 deadline
 *
 * 使用无符号比较处理计数器回绕:
 * 当 (当前值 - deadline) 的最高位为 0 时, 表示还未到达。
 *
 * @param deadline 目标计数值 (STK_CNTL)
 */
static inline __attribute__((always_inline)) void wait_until(uint32_t deadline)
{
    while (((uint32_t)(STK_CNTL - deadline) >> 31) != 0u) { }
}

#if BMCU_ONLINE_LED_FILAMENT_RGB
/**
 * @brief 视频缩放: 将 8 位值乘以缩放因子
 *
 * 使用 (v * scale + 255) >> 8 公式, 比纯整数除法更平滑,
 * 避免低值时出现死区。类似于 FastLED 的 scale8_video。
 *
 * @param v 原始值 (0-255)
 * @param scale 缩放因子 (0-255, 255=不缩放)
 * @return 缩放后的值 (0-255)
 */
static inline __attribute__((always_inline)) uint8_t scale8_video(uint8_t v, uint8_t scale)
{
    return (uint8_t)(((uint16_t)v * (uint16_t)scale + 255u) >> 8);
}

/**
 * @brief Gamma 校正查找表 (256 项)
 *
 * 将线性亮度值映射到感知均匀的亮度值。
 * 公式: output = (input/255)^2.2 × 255
 * 低亮度区域映射更精细, 高亮度区域压缩,
 * 匹配人眼对亮度的非线性感知。
 */
static const uint8_t kGamma8[256] =
{
      0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
      0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   1u,   1u,   1u,   1u,   1u,   1u,   1u,   2u,
      2u,   2u,   2u,   2u,   2u,   3u,   3u,   3u,   3u,   4u,   4u,   4u,   4u,   5u,   5u,   5u,
      6u,   6u,   6u,   7u,   7u,   7u,   8u,   8u,   8u,   9u,   9u,  10u,  10u,  10u,  11u,  11u,
     12u,  12u,  13u,  13u,  14u,  14u,  15u,  15u,  16u,  16u,  17u,  17u,  18u,  18u,  19u,  20u,
     20u,  21u,  21u,  22u,  23u,  23u,  24u,  25u,  25u,  26u,  27u,  28u,  28u,  29u,  30u,  31u,
     31u,  32u,  33u,  34u,  35u,  35u,  36u,  37u,  38u,  39u,  40u,  41u,  42u,  43u,  44u,  45u,
     46u,  47u,  48u,  49u,  50u,  51u,  52u,  54u,  55u,  56u,  57u,  58u,  60u,  61u,  62u,  64u,
     65u,  66u,  68u,  69u,  70u,  72u,  73u,  75u,  76u,  78u,  79u,  81u,  82u,  84u,  85u,  87u,
     89u,  90u,  92u,  94u,  95u,  97u,  99u, 100u, 102u, 104u, 106u, 108u, 109u, 111u, 113u, 115u,
    117u, 119u, 121u, 123u, 125u, 127u, 129u, 131u, 133u, 135u, 137u, 139u, 141u, 143u, 145u, 148u,
    150u, 152u, 154u, 156u, 158u, 161u, 163u, 165u, 167u, 170u, 172u, 174u, 177u, 179u, 181u, 184u,
    186u, 189u, 191u, 193u, 196u, 198u, 201u, 203u, 206u, 208u, 211u, 213u, 216u, 219u, 221u, 224u,
    226u, 229u, 232u, 234u, 237u, 240u, 242u, 245u, 248u, 250u, 253u, 255u, 255u, 255u, 255u, 255u,
    255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
    255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u
};
#endif

/**
 * @brief 初始化 WS2812 驱动器
 *
 * 流程:
 * 1. 计算复位脉冲时间 (100us, datasheet 要求 ≥50us)
 * 2. 限制 LED 数量不超过 MAX_NUM
 * 3. 使能 GPIO 时钟并配置为推挽输出 (50MHz)
 * 4. 拉低数据线, 清空颜色缓冲区
 *
 * @param _num LED 数量 (超过 MAX_NUM 则截断)
 * @param _port GPIO 端口
 * @param _pin GPIO 引脚号
 */
void WS2812_class::init(uint8_t _num, GPIO_TypeDef* _port, uint16_t _pin)
{
    dirty = false;

    // 首次调用时计算复位脉冲 tick 数
    if (!g_ws2812_rst_ticks) {
        g_ws2812_rst_ticks = 100u * time_hw_ticks_per_us(); // 100us
        if (!g_ws2812_rst_ticks) g_ws2812_rst_ticks = 1u;
    }

    if (_num > MAX_NUM) _num = MAX_NUM;

    num  = _num;
    port = _port;
    pin  = _pin;

    enable_gpio_clock(port);

    // 配置为推挽输出, 最大速度 50MHz
    GPIO_InitTypeDef gi = {0};
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    gi.GPIO_Mode  = GPIO_Mode_Out_PP;
    gi.GPIO_Pin   = pin;
    GPIO_Init(port, &gi);

    RGB_L(port, pin); // 初始拉低数据线
    clear();          // 清空颜色缓冲区
}

/**
 * @brief 清空颜色缓冲区 (全部设为黑色)
 *
 * 清零 GRB 缓冲区、在线 RGB 缓存和耗材模式标志,
 * 设置脏标记等待下次发送。
 */
void WS2812_class::clear(void)
{
    for (uint32_t i = 0; i < (uint32_t)num; i++) last_grb[i] = 0u;
    for (uint32_t i = 0; i < (uint32_t)num; i++) last_online_raw_rgb[i] = 0u;
    for (uint32_t i = 0; i < (uint32_t)num; i++) last_online_is_filament[i] = 0u;
    dirty = true;
}

/**
 * @brief 发送复位脉冲: 拉低数据线至少 50us
 *
 * 实际使用 100us 以留有裕量。复位脉冲使 WS2812B
 * 锁存之前接收的像素数据并准备接收新数据。
 */
void WS2812_class::RST(void)
{
    RGB_L(port, pin);
    delayTicks32(g_ws2812_rst_ticks);
}

/**
 * @brief 将颜色缓冲区数据发送到 WS2812B LED 灯链
 *
 * 关键实现细节:
 * 1. 检查 dirty 标志, 无变化则跳过
 * 2. 关中断以确保精确时序
 * 3. 对齐到 STK 计数器边界 (确保起始时间一致)
 * 4. 逐 LED、逐位发送 GRB 数据 (MSB 优先)
 * 5. 每位根据 0/1 决定高电平持续时间
 * 6. 发送完成后清除脏标记, 恢复中断
 * 7. 发送复位脉冲锁存数据
 */
void WS2812_class::updata(void)
{
    if (!dirty) return;

    GPIO_TypeDef* const p = port;
    const uint32_t      m = (uint32_t)pin;

    uint32_t irq = irq_save_wch(); // 关中断, 保证时序精确

    // 对齐到 STK 计数器新 tick 边界
    uint32_t base = STK_CNTL;
    while (STK_CNTL == base) { } // 等待当前 tick 结束
    base = STK_CNTL;

    for (uint32_t led = 0; led < (uint32_t)num; led++)
    {
        // GRB 格式, MSB 优先发送
        uint32_t v = last_grb[led];

        for (uint32_t k = 0; k < 24u; k++)
        {
            // 根据最高位决定高电平时间: 1=T1H(长), 0=T0H(短)
            const uint32_t th = (v & 0x800000u) ? WS2812_T1H_TICKS : WS2812_T0H_TICKS;
            v <<= 1; // 左移准备下一位

            RGB_H(p, m);       // 拉高数据线
            base += th;         // 累加高电平时间
            wait_until(base);   // 忙等待到达目标时间

            RGB_L(p, m);                   // 拉低数据线
            base += (WS2812_TBIT_TICKS - th); // 累加剩余低电平时间
            wait_until(base);              // 忙等待位周期结束
        }
    }

    dirty = false;
    irq_restore_wch(irq); // 恢复中断
    RST();                 // 发送复位脉冲锁存数据
}

/**
 * @brief 直接设置指定 LED 的 RGB 颜色
 *
 * 将 RGB 转换为 GRB 格式 (WS2812B 标准), 与缓冲区比较:
 * 若颜色未变化则跳过, 避免不必要的发送。
 *
 * @param R 红色分量 (0-255)
 * @param G 绿色分量 (0-255)
 * @param B 蓝色分量 (0-255)
 * @param index LED 索引 (0 ~ num-1)
 */
void WS2812_class::set_RGB(uint8_t R, uint8_t G, uint8_t B, uint8_t index)
{
    if (index >= num) return;

    const uint32_t packed = ((uint32_t)G << 16) | ((uint32_t)R << 8) | (uint32_t)B; // GRB 打包
    if (last_grb[index] == packed) return; // 颜色未变, 跳过

    last_grb[index] = packed;
    dirty = true;
}

/**
 * @brief 在线显示模式下设置 LED 颜色
 *
 * 两种模式:
 * 1. 非耗材模式 (filament=false): 直接调用 set_RGB
 * 2. 耗材模式 (filament=true):
 *    - 若未启用 BMCU_ONLINE_LED_FILAMENT_RGB, 退化为直接模式
 *    - 否则执行色彩校正管线:
 *      a. Gamma 校正: 线性 → 感知均匀
 *      b. 通道缩放: 补偿 LED 色温 (G×0.69, B×0.94)
 *      c. 亮度限制: 全通道 ×0.125 (最大亮度 32/255)
 *      d. 缓存去重: 相同原始 RGB 不重复计算
 *
 * @param R 红色分量 (0-255)
 * @param G 绿色分量 (0-255)
 * @param B 蓝色分量 (0-255)
 * @param index LED 索引 (0 ~ num-1)
 * @param filament 是否为耗材颜色模式
 */
void WS2812_class::set_RGB_online(uint8_t R, uint8_t G, uint8_t B, uint8_t index, bool filament)
{
    if (index >= num) return;

    // 非耗材模式: 直接设置, 清除耗材标志
    if (!filament)
    {
        last_online_is_filament[index] = 0u;
        set_RGB(R, G, B, index);
        return;
    }

#if !BMCU_ONLINE_LED_FILAMENT_RGB
    // 未启用耗材色彩校正: 退化为直接模式
    (void)filament;
    set_RGB(R, G, B, index);
#else
    // 打包原始 RGB 用于去重比较
    const uint32_t raw = ((uint32_t)R << 16) | ((uint32_t)G << 8) | (uint32_t)B;

    // 缓存命中: 原始 RGB 相同且已是耗材模式, 跳过
    if (last_online_is_filament[index] && (last_online_raw_rgb[index] == raw))
        return;

    last_online_is_filament[index] = 1u;
    last_online_raw_rgb[index]     = raw;

    // 色彩校正管线:
    // 第1步: Gamma 校正 (线性亮度 → 感知均匀)
    uint8_t rr = kGamma8[R];
    uint8_t gg = kGamma8[G];
    uint8_t bb = kGamma8[B];

    // 第2步: LED 色温补偿缩放
    // 绿色通道衰减到 69% (176/255), 补偿绿色 LED 过亮
    gg = scale8_video(gg, 176u);
    // 蓝色通道衰减到 94% (240/255), 补偿蓝色 LED 略亮
    bb = scale8_video(bb, 240u);

    // 第3步: 整体亮度限制 (最大 12.5% = 32/255)
    // 防止耗材颜色过亮, 同时降低功耗
    rr = scale8_video(rr, 32u);
    gg = scale8_video(gg, 32u);
    bb = scale8_video(bb, 32u);

    set_RGB(rr, gg, bb, index);
#endif
}
