/**
 * @file ADC_DMA.cpp
 * @brief ADC + DMA 多通道模拟采样实现
 *
 * 使用 CH32V203 的双 ADC (ADC1+ADC2) 同步采样模式:
 * - ADC1 为主设备, ADC2 为从设备
 * - 8 个模拟通道 (PA0-PA7) 同步采样
 * - DMA1_Channel1 循环搬运, 半传输+全传输中断
 * - 环形滤波器: 4 个块 × 32 个采样点/块 = 128 次采样平均
 *
 * 数据流: ADC → DMA缓冲区(512 uint32) → 半块处理 → 环形累加 → 浮点转换
 */

#include "ADC_DMA.h"
#include "hal/time_hw.h"
#include "ch32v20x_adc.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_rcc.h"
#include <stdint.h>

/** @brief ADC 通道数: 8 个模拟输入通道 (PA0-PA7) */
static constexpr uint32_t kCh      = 8;
/** @brief 每块采样数: 每个块包含 32 个采样周期 (双 ADC 同步, 每周期产生 2 个样本) */
static constexpr uint32_t kBlock   = 32;
/** @brief 环形滤波器块数: 4 块用于滑动平均 */
static constexpr uint32_t kNBlocks = 4;
/** @brief DMA 缓冲区总长度: 8通道 × 32样本 × 2(双ADC) = 512 个 uint32 */
static constexpr uint32_t kBufLen  = (kCh * kBlock * 2);
/** @brief 半缓冲区长度: 256 个 uint32 (半传输中断触发点) */
static constexpr uint32_t kHalfLen = (kBufLen / 2);

/** @brief ADC DMA 初始化完成标志 */
static bool g_adc_dma_inited = false;
/**
 * @brief 检查 ADC DMA 是否已完成初始化
 * @return true 已初始化
 */
bool ADC_DMA_is_inited() { return g_adc_dma_inited; }

/** @brief DMA 缓冲区: 循环模式, 存储 ADC 采样原始数据 (512 个 uint32) */
static volatile uint32_t g_dma_buf[kBufLen] __attribute__((aligned(4)));

/** @brief 环形滤波器: 4 块 × 8 通道, 存储每块的累加和 */
static uint32_t g_ring_sum[kNBlocks][kCh];
/** @brief 全局累加器: 所有块的通道累加和总和 */
static uint32_t g_acc_sum[kCh];
/** @brief 环形缓冲区写入索引 (0 ~ kNBlocks-1) */
static uint8_t  g_ring_idx = 0;
/** @brief 已填充的块数 (0 ~ kNBlocks), 用于判断数据是否稳定 */
static uint8_t  g_blocks_filled = 0;

/** @brief 双缓冲浮点结果: [0] 和 [1] 交替作为读/写缓冲区 */
static float            g_v[2][kCh] __attribute__((aligned(4)));
/** @brief 浮点结果读缓冲区索引 (0 或 1) */
static volatile uint8_t g_v_rd = 0;
/** @brief 累加器脏标志: 1 表示有新数据需要转换为浮点 */
static volatile uint8_t g_acc_dirty = 0;

/** @brief 缩放系数: 将 ADC 累加和转换为伏特值
 *  公式: voltage = acc_sum × scale
 *  scale = 3.3V / (8190 × 采样数), 8190 是 13 位 ADC 有效范围 */
static constexpr float kScale32  = 3.3f / (8190.0f *  32.0f);  ///< 1块32样本的缩放
static constexpr float kScale64  = 3.3f / (8190.0f *  64.0f);  ///< 2块64样本的缩放
static constexpr float kScale96  = 3.3f / (8190.0f *  96.0f);  ///< 3块96样本的缩放
static constexpr float kScale128 = 3.3f / (8190.0f * 128.0f);  ///< 4块128样本的缩放
/** @brief 当前使用的缩放系数 (随块数递增逐步调整) */
static float g_scale = kScale32;

/**
 * @brief RISC-V 内存屏障: 确保 DMA 内存访问顺序
 *
 * 使用 fence iorw,iorw 指令确保之前的所有 IO 和内存写入
 * 在后续 IO/内存操作之前完成。DMA 和 CPU 并发访问时必需。
 */
static inline __attribute__((always_inline)) void adc_dma_barrier()
{
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

/**
 * @brief 编译器屏障: 防止编译器重排跨此点的内存访问
 *
 * 不生成任何机器指令, 仅阻止编译器优化。
 */
static inline __attribute__((always_inline)) void adc_dma_compiler_barrier()
{
    __asm__ volatile("" ::: "memory");
}

/**
 * @brief 将 PA0-PA7 配置为模拟输入模式
 *
 * 使能 GPIOA 时钟, 设置全部 8 个引脚为模拟输入。
 * 模拟输入模式下施密特触发器禁用, 降低功耗。
 */
void ADC_DMA_gpio_analog()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Mode  = GPIO_Mode_AIN;  // 模拟输入模式
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                      GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);
}

/**
 * @brief 重置滤波器: 清空所有环形缓冲区和累加器
 *
 * 将环形索引归零, 块计数归零, 缩放系数回到初始值,
 * 清零所有通道的累加和与环形和。
 */
static inline void filter_reset()
{
    g_ring_idx = 0;
    g_blocks_filled = 0;
    g_scale = kScale32;

    for (uint32_t ch = 0; ch < kCh; ch++) g_acc_sum[ch] = 0;

    for (uint32_t b = 0; b < kNBlocks; b++)
        for (uint32_t ch = 0; ch < kCh; ch++)
            g_ring_sum[b][ch] = 0;

    for (uint32_t ch = 0; ch < kCh; ch++)
    {
        g_v[0][ch] = 0.0f;
        g_v[1][ch] = 0.0f;
    }

    g_v_rd = 0;
    g_acc_dirty = 0;
}

/**
 * @brief 从打包的 32 位字中提取两个 ADC 样本并求和
 *
 * 双 ADC 同步模式下, 每次采样产生两个 13 位值,
 * 打包在一个 32 位字中: [高16位=ADC2样本][低16位=ADC1样本]
 *
 * @param w 打包的 ADC 数据字
 * @return 两个 16 位样本之和 (0 ~ 16380)
 */
static inline __attribute__((always_inline)) uint32_t adc_pair_sum(uint32_t w)
{
    const uint32_t a1 = (uint16_t)(w & 0xFFFFu);  // ADC1 样本 (低16位)
    const uint32_t a2 = (uint16_t)(w >> 16);       // ADC2 样本 (高16位)
    return a1 + a2;
}

/**
 * @brief 处理半缓冲区数据并更新环形滤波器
 *
 * 对 32 个采样行 (每行 8 个打包字) 逐通道累加,
 * 然后将新块的和写入环形缓冲区并更新全局累加器。
 * 使用增量更新: acc_sum = acc_sum - old_block + new_block
 *
 * @param p_half 指向半缓冲区起始 (256 个 uint32)
 */
static inline void process_half_update_filter(const uint32_t* p_half)
{
    // 手动展开 8 个通道的累加, 提升性能
    uint32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0, s7 = 0;

    const uint32_t* __restrict row = p_half;
    for (uint32_t s = 0; s < kBlock; s++) // 32 个采样行
    {
        s0 += adc_pair_sum(row[0]); // 通道 0
        s1 += adc_pair_sum(row[1]); // 通道 1
        s2 += adc_pair_sum(row[2]); // 通道 2
        s3 += adc_pair_sum(row[3]); // 通道 3
        s4 += adc_pair_sum(row[4]); // 通道 4
        s5 += adc_pair_sum(row[5]); // 通道 5
        s6 += adc_pair_sum(row[6]); // 通道 6
        s7 += adc_pair_sum(row[7]); // 通道 7
        row += kCh; // 移动到下一行 (8 个 uint32)
    }

    // 增量更新环形滤波器
    const uint8_t idx = g_ring_idx;
    {
        uint32_t o;
        // 通道 0: 减去旧块, 加上新块
        o = g_ring_sum[idx][0]; g_ring_sum[idx][0] = s0; g_acc_sum[0] = g_acc_sum[0] - o + s0;
        o = g_ring_sum[idx][1]; g_ring_sum[idx][1] = s1; g_acc_sum[1] = g_acc_sum[1] - o + s1;
        o = g_ring_sum[idx][2]; g_ring_sum[idx][2] = s2; g_acc_sum[2] = g_acc_sum[2] - o + s2;
        o = g_ring_sum[idx][3]; g_ring_sum[idx][3] = s3; g_acc_sum[3] = g_acc_sum[3] - o + s3;
        o = g_ring_sum[idx][4]; g_ring_sum[idx][4] = s4; g_acc_sum[4] = g_acc_sum[4] - o + s4;
        o = g_ring_sum[idx][5]; g_ring_sum[idx][5] = s5; g_acc_sum[5] = g_acc_sum[5] - o + s5;
        o = g_ring_sum[idx][6]; g_ring_sum[idx][6] = s6; g_acc_sum[6] = g_acc_sum[6] - o + s6;
        o = g_ring_sum[idx][7]; g_ring_sum[idx][7] = s7; g_acc_sum[7] = g_acc_sum[7] - o + s7;
    }

    // 推进环形索引
    uint8_t ni = (uint8_t)(idx + 1u);
    if (ni >= kNBlocks) ni = 0u;
    g_ring_idx = ni;

    // 随填充块数增加, 逐步调整缩放系数
    if (g_blocks_filled < kNBlocks)
    {
        g_blocks_filled++;
        if      (g_blocks_filled == 1u) g_scale = kScale32;   // 1块: 32样本平均
        else if (g_blocks_filled == 2u) g_scale = kScale64;   // 2块: 64样本平均
        else if (g_blocks_filled == 3u) g_scale = kScale96;   // 3块: 96样本平均
        else                            g_scale = kScale128;  // 4块: 128样本平均 (完全稳定)
    }

    adc_dma_compiler_barrier(); // 防止编译器重排脏标志写入
    g_acc_dirty = 1u;           // 标记有新数据需要转换
}

/**
 * @brief 轮询处理 DMA 中断标志并更新滤波器
 *
 * 检查 DMA1_Channel1 的三种中断标志:
 * - TE (传输错误): 清除标志, 重置滤波器
 * - HT (半传输): 处理前半缓冲区数据
 * - TC (全传输): 处理后半缓冲区数据
 *
 * 最多处理 4 个事件, 防止无限循环。
 */
void ADC_DMA_poll()
{
    for (int guard = 0; guard < 4; guard++)
    {
        const uint32_t flags = DMA1->INTFR;

        // 传输错误: 重置滤波器, 从头开始
        if (flags & DMA1_FLAG_TE1)
        {
            DMA1->INTFCR = (DMA1_FLAG_TE1 | DMA1_FLAG_HT1 | DMA1_FLAG_TC1);
            adc_dma_barrier();
            filter_reset();
            continue;
        }

        // 半传输: 前半缓冲区 (0 ~ kHalfLen-1) 数据就绪
        if (flags & DMA1_FLAG_HT1)
        {
            DMA1->INTFCR = DMA1_FLAG_HT1;
            adc_dma_barrier();
            const uint32_t* p = (const uint32_t*)(const void*)&g_dma_buf[0];
            process_half_update_filter(p);
            continue;
        }

        // 全传输: 后半缓冲区 (kHalfLen ~ kBufLen-1) 数据就绪
        if (flags & DMA1_FLAG_TC1)
        {
            DMA1->INTFCR = DMA1_FLAG_TC1;
            adc_dma_barrier();
            const uint32_t* p = (const uint32_t*)(const void*)&g_dma_buf[kHalfLen];
            process_half_update_filter(p);
            continue;
        }

        break; // 无待处理事件
    }
}

/**
 * @brief 获取最新滤波后的 8 通道电压值
 *
 * 先调用 poll() 处理新数据, 若累加器有更新,
 * 则将 8 通道的累加和乘以缩放系数转换为伏特值,
 * 写入备用缓冲区, 然后切换读索引 (双缓冲)。
 *
 * @return 指向 8 个 float 的静态数组 (单位: V)
 */
const float *ADC_DMA_get_value()
{
    ADC_DMA_poll();

    if (g_acc_dirty)
    {
        const float scale = g_scale;
        const uint8_t wr = (uint8_t)(g_v_rd ^ 1u); // 写入另一个缓冲区
        float* out = g_v[wr];

        // 8 通道并行转换: 累加和 × 缩放系数 = 电压值
        out[0] = (float)g_acc_sum[0] * scale;
        out[1] = (float)g_acc_sum[1] * scale;
        out[2] = (float)g_acc_sum[2] * scale;
        out[3] = (float)g_acc_sum[3] * scale;
        out[4] = (float)g_acc_sum[4] * scale;
        out[5] = (float)g_acc_sum[5] * scale;
        out[6] = (float)g_acc_sum[6] * scale;
        out[7] = (float)g_acc_sum[7] * scale;

        adc_dma_compiler_barrier();
        g_v_rd = wr;         // 原子切换读缓冲区
        g_acc_dirty = 0u;    // 清除脏标志
    }

    return g_v[g_v_rd];
}

/**
 * @brief 重置滤波器并清除所有 DMA 中断标志
 *
 * 在通道切换或需要重新稳定采样数据时调用。
 * 先清除 DMA 标志, 再重置内部滤波器状态。
 */
void ADC_DMA_filter_reset()
{
    DMA1->INTFCR = (DMA1_FLAG_GL1 | DMA1_FLAG_HT1 | DMA1_FLAG_TC1 | DMA1_FLAG_TE1);
    adc_dma_barrier();
    filter_reset();
}

/**
 * @brief 检查滤波器是否已填满全部 4 个块
 * @return true 数据已完全稳定
 */
bool ADC_DMA_ready()
{
    return (g_blocks_filled >= kNBlocks);
}

/**
 * @brief 阻塞等待滤波器填满
 *
 * 在 ADC 初始化后或滤波器重置后调用,
 * 最多等待 2 秒。期间不断调用 poll() 处理 DMA 数据。
 */
void ADC_DMA_wait_full()
{
    const uint32_t t0 = time_ticks32();
    const uint32_t tout = ms_to_ticks32(2000u); // 2 秒超时

    while (g_blocks_filled < kNBlocks)
    {
        ADC_DMA_poll();
        if ((uint32_t)(time_ticks32() - t0) > tout) break; // 超时退出
        delay(1);
    }
}

/**
 * @brief ADC 校准: 执行复位校准和自校准
 *
 * CH32V203 ADC 上电后需要校准以消除偏移误差。
 * 先执行复位校准, 等待完成; 再执行自校准, 等待完成。
 *
 * @param a ADC 外设指针 (ADC1 或 ADC2)
 */
static inline void adc_calibrate(ADC_TypeDef* a)
{
    ADC_Cmd(a, ENABLE);            // 使能 ADC
    ADC_BufferCmd(a, DISABLE);     // 禁用模拟看门狗缓冲

    ADC_ResetCalibration(a);       // 启动复位校准
    while (ADC_GetResetCalibrationStatus(a)) {} // 等待复位校准完成
    ADC_StartCalibration(a);       // 启动自校准
    while (ADC_GetCalibrationStatus(a)) {}      // 等待自校准完成
}

/**
 * @brief 完整初始化 ADC + DMA 外设
 *
 * 初始化流程:
 * 1. 使能 GPIOA/DMA1/ADC1/ADC2 时钟
 * 2. 配置 ADC 时钟分频 (PCLK2/8)
 * 3. 配置 PA0-PA7 为模拟输入
 * 4. 初始化 DMA: 循环模式, 字传输, 高优先级
 * 5. 配置 ADC1 为同步主设备, ADC2 为从设备
 * 6. 配置 8 个规则通道, 采样时间 71.5 周期
 * 7. 校准 ADC1 和 ADC2
 * 8. 使能 DMA, 启动转换
 * 9. 等待首次数据到达 (最多 350ms)
 * 10. 重置滤波器并等待数据稳定
 *
 * 若已初始化, 则仅重置 GPIO 和滤波器。
 */
void ADC_DMA_init()
{
    // 已初始化: 仅重置 GPIO 和滤波器
    if (g_adc_dma_inited)
    {
        ADC_DMA_gpio_analog();
        ADC_DMA_filter_reset();
        ADC_DMA_wait_full();
        return;
    }

    // 使能外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8); // ADC 时钟 = PCLK2 / 8 = 9MHz

    ADC_DMA_gpio_analog();

    // 初始化 DMA 缓冲区为全 0xFFFFFFFF (空白值)
    for (uint32_t i = 0; i < kBufLen; i++) g_dma_buf[i] = 0xFFFFFFFFu;
    filter_reset();

    // 配置 DMA1_Channel1
    DMA_DeInit(DMA1_Channel1);
    DMA_Cmd(DMA1_Channel1, DISABLE);

    // ADC1 规则数据寄存器地址 (0x4001244C)
    constexpr uint32_t RDATAR_ADDRESS = 0x4001244Cu;

    DMA_InitTypeDef dma = {0};
    dma.DMA_PeripheralBaseAddr = RDATAR_ADDRESS;    // 外设地址: ADC1 规则数据寄存器
    dma.DMA_MemoryBaseAddr     = (uint32_t)g_dma_buf; // 内存地址: DMA 缓冲区
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC; // 方向: 外设→内存
    dma.DMA_BufferSize         = kBufLen;           // 传输总数: 512 字
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable; // 外设地址不递增
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;      // 内存地址递增
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // 外设数据: 32位
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;    // 内存数据: 32位
    dma.DMA_Mode               = DMA_Mode_Circular;    // 循环模式: 缓冲区满后自动回到开头
    dma.DMA_Priority           = DMA_Priority_High;    // 高优先级
    dma.DMA_M2M                = DMA_M2M_Disable;      // 非内存到内存模式
    DMA_Init(DMA1_Channel1, &dma);

    DMA_SetCurrDataCounter(DMA1_Channel1, kBufLen);

    // 清除所有 DMA 中断标志, 禁用中断 (使用轮询模式)
    DMA1->INTFCR = (DMA1_FLAG_GL1 | DMA1_FLAG_HT1 | DMA1_FLAG_TC1 | DMA1_FLAG_TE1);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE, DISABLE);

    // 复位 ADC1 和 ADC2
    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    // 配置 ADC1: 同步规则模式, 扫描, 连续, 8通道
    ADC_InitTypeDef a1 = {0};
    a1.ADC_Mode               = ADC_Mode_RegSimult;     // 同步规则模式 (双ADC)
    a1.ADC_ScanConvMode       = ENABLE;                 // 扫描模式 (多通道)
    a1.ADC_ContinuousConvMode = ENABLE;                 // 连续转换模式
    a1.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None; // 软件触发
    a1.ADC_DataAlign          = ADC_DataAlign_Right;    // 右对齐
    a1.ADC_NbrOfChannel       = 8;                      // 8 个转换通道
    a1.ADC_OutputBuffer       = ADC_OutputBuffer_Disable; // 禁用输出缓冲
    a1.ADC_Pga                = ADC_Pga_1;              // 增益 = 1 (无放大)
    ADC_Init(ADC1, &a1);

    // 配置 ADC2: 独立模式 (从设备由 ADC1 触发)
    ADC_InitTypeDef a2 = {0};
    a2.ADC_Mode               = ADC_Mode_Independent;   // 独立模式 (同步从设备)
    a2.ADC_ScanConvMode       = ENABLE;
    a2.ADC_ContinuousConvMode = ENABLE;
    a2.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    a2.ADC_DataAlign          = ADC_DataAlign_Right;
    a2.ADC_NbrOfChannel       = 8;
    a2.ADC_OutputBuffer       = ADC_OutputBuffer_Disable;
    a2.ADC_Pga                = ADC_Pga_1;
    ADC_Init(ADC2, &a2);

    // 配置 8 个规则通道: ADC1 和 ADC2 都采样 PA0-PA7
    for (int i = 0; i < 8; i++)
    {
        ADC_RegularChannelConfig(ADC1, (uint8_t)i, (uint8_t)(i + 1), ADC_SampleTime_71Cycles5);
        ADC_RegularChannelConfig(ADC2, (uint8_t)i, (uint8_t)(i + 1), ADC_SampleTime_71Cycles5);
    }

    // 校准两个 ADC
    adc_calibrate(ADC1);
    adc_calibrate(ADC2);

    // 使能 ADC1 的 DMA 请求
    ADC_DMACmd(ADC1, ENABLE);

    // 启动 DMA
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // 启动 ADC 转换 (ADC2 先启动, 作为从设备等待 ADC1 同步)
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // 等待首次数据到达 (最多 350ms)
    uint32_t t0 = time_ticks32();
    const uint32_t warm = ms_to_ticks32(350u);

    while ((uint32_t)(time_ticks32() - t0) < warm)
    {
        const uint32_t f = DMA1->INTFR;
        if (f & (DMA1_FLAG_HT1 | DMA1_FLAG_TC1 | DMA1_FLAG_TE1)) break; // 有中断发生
        if (g_dma_buf[0] != 0xFFFFFFFFu) break; // 首个样本已到达
        delay(1);
    }

    // 检查是否成功: 缓冲区非空白或有中断发生
    {
        const uint32_t f = DMA1->INTFR;
        g_adc_dma_inited =
            (g_dma_buf[0] != 0xFFFFFFFFu) ||
            (f & (DMA1_FLAG_HT1 | DMA1_FLAG_TC1));
    }

    // 清除所有中断标志
    DMA1->INTFCR = (DMA1_FLAG_GL1 | DMA1_FLAG_HT1 | DMA1_FLAG_TC1 | DMA1_FLAG_TE1);

    // 重置滤波器并等待数据完全稳定
    if (g_adc_dma_inited)
    {
        ADC_DMA_filter_reset();
        ADC_DMA_wait_full();
    }
}
