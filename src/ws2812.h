#pragma once
#include <stdint.h>
#include "ch32v20x.h"

/**
 * @file ws2812.h
 * @brief WS2812B RGB LED 驱动接口
 *
 * 使用位操作 (bit-bang) 方式驱动 WS2812B RGB LED 灯带。
 * 通过 SysTick 定时器精确控制 GPIO 高低电平时间,
 * 实现 WS2812B 单线串行协议。
 *
 * 支持最多 4 个 LED (MAX_NUM), 用于 AMS 耗材槽位指示。
 *
 * 协议时序 (WS2812B):
 *   - T0H = 0.4us ±150ns (逻辑0)
 *   - T1H = 0.85us ±150ns (逻辑1)
 *   - 位周期 = 1.25us ±600ns
 *   - 复位脉冲 ≥ 50us
 */

/** @brief 在线模式下是否对耗材颜色应用 Gamma 校正和色彩缩放 */
#ifndef BMCU_ONLINE_LED_FILAMENT_RGB
#define BMCU_ONLINE_LED_FILAMENT_RGB 0
#endif

/**
 * @brief WS2812B LED 驱动器类
 *
 * 管理一个 GPIO 引脚上的 WS2812B LED 灯链。
 * 使用 GRB 颜色顺序 (WS2812B 标准), 支持:
 * - 直接设置 RGB 颜色 (set_RGB)
 * - 在线模式带 Gamma 校正 (set_RGB_online)
 * - 脏标记优化: 仅在数据变化时才发送
 */
class WS2812_class
{
public:
    /** @brief 每条灯链最大支持的 LED 数量 */
    static constexpr uint8_t MAX_NUM = 4;

    /**
     * @brief 初始化 WS2812 驱动
     * @param num LED 数量 (1~4)
     * @param port GPIO 端口 (GPIOA/B/C/D)
     * @param pin GPIO 引脚号
     */
    void init(uint8_t num, GPIO_TypeDef* port, uint16_t pin);

    /**
     * @brief 清空颜色缓冲区 (全部设为黑色)
     *
     * 清空 GRB 缓冲区和在线缓存, 设置脏标记。
     * 需要调用 updata() 才会实际发送到 LED。
     */
    void clear(void);

    /**
     * @brief 发送复位脉冲 (拉低 ≥50us)
     *
     * 使 LED 锁存之前接收的数据。
     */
    void RST(void);

    /**
     * @brief 将缓冲区数据发送到 LED 灯链
     *
     * 仅在 dirty 标志为 true 时发送。发送过程中关中断,
     * 使用 SysTick 精确计时, 逐位发送 GRB 数据。
     * 发送完成后自动发送复位脉冲。
     */
    void updata(void);

    /**
     * @brief 设置指定 LED 的 RGB 颜色 (直接模式)
     *
     * 将 RGB 转换为 GRB 格式存储。若颜色未变化则跳过。
     *
     * @param R 红色分量 (0-255)
     * @param G 绿色分量 (0-255)
     * @param B 蓝色分量 (0-255)
     * @param index LED 索引 (0 ~ num-1)
     */
    void set_RGB(uint8_t R, uint8_t G, uint8_t B, uint8_t index);

    /**
     * @brief 设置指定 LED 的颜色 (在线显示模式)
     *
     * 当 filament=false 时等同于 set_RGB。
     * 当 filament=true 且 BMCU_ONLINE_LED_FILAMENT_RGB 启用时,
     * 对颜色应用 Gamma 校正和 LED 色彩特性补偿:
     *   1. Gamma 校正 (非线性亮度映射)
     *   2. 通道缩放 (补偿 LED 色温偏差)
     *   3. 整体亮度限制 (防止过流)
     *
     * @param R 红色分量 (0-255)
     * @param G 绿色分量 (0-255)
     * @param B 蓝色分量 (0-255)
     * @param index LED 索引 (0 ~ num-1)
     * @param filament 是否为耗材颜色模式 (默认 false)
     */
    void set_RGB_online(uint8_t R, uint8_t G, uint8_t B, uint8_t index, bool filament = false);

    /**
     * @brief 检查是否有待发送的颜色数据
     * @return true 有脏数据需要发送
     */
    inline bool is_dirty() const { return dirty; }

private:
    /** @brief GPIO 端口指针 */
    GPIO_TypeDef* port = nullptr;
    /** @brief GPIO 引脚掩码 */
    uint16_t      pin  = 0;
    /** @brief LED 数量 */
    uint8_t       num  = 0;

    /**
     * @brief GRB 颜色缓冲区 (32位打包: [23:16]=G, [15:8]=R, [7:0]=B)
     *
     * WS2812B 使用 GRB 颜色顺序, 每个 LED 占 24 位。
     */
    uint32_t last_grb[MAX_NUM] = {0u, 0u, 0u, 0u};

    /**
     * @brief 在线模式原始 RGB 缓存 (用于去重比较)
     *
     * 仅在 filament 模式下使用, 存储未经过 Gamma 校正的原始 RGB 值,
     * 避免相同颜色重复计算和发送。
     */
    uint32_t last_online_raw_rgb[MAX_NUM]   = {0u, 0u, 0u, 0u}; // RGB packed
    /** @brief 各 LED 是否处于在线耗材模式标志 */
    uint8_t  last_online_is_filament[MAX_NUM] = {0u, 0u, 0u, 0u};

    /** @brief 脏标记: 为 true 时表示缓冲区有变化需要发送 */
    bool dirty = false;
};
