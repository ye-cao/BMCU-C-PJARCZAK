#pragma once

#include <stdbool.h>

/**
 * @file ADC_DMA.h
 * @brief ADC + DMA 多通道模拟采样接口
 *
 * 使用 CH32V203 的 ADC1/ADC2 双 ADC 同步采样模式,
 * 通过 DMA1_Channel1 自动搬运数据到内存缓冲区。
 * 支持 8 通道同时采样 (PA0-PA7), 用于电机电流和电压检测。
 *
 * 采样流程:
 *   ADC1+ADC2 同步采样 → DMA 循环搬运 → 半缓冲中断处理
 *   → 环形滤波器求和 → 转换为浮点电压值
 */

/**
 * @brief 初始化 ADC 和 DMA 外设
 *
 * 配置 ADC1/ADC2 为同步规则扫描模式, DMA 循环模式。
 * 首次调用完成全部硬件初始化, 后续调用仅重置滤波器并等待稳定。
 * 初始化后需要等待约 350ms 预热时间。
 */
void  ADC_DMA_init(void);

/**
 * @brief 检查 ADC DMA 是否已完成初始化
 * @return true 已初始化且数据可用
 */
bool  ADC_DMA_is_inited(void);

/**
 * @brief 将 PA0-PA7 配置为模拟输入模式
 *
 * 在需要重新配置 GPIO 或低功耗恢复后调用。
 */
void  ADC_DMA_gpio_analog(void);

/**
 * @brief 轮询处理 DMA 半传输/全传输/错误中断标志
 *
 * 非中断驱动模式下由主循环定期调用,
 * 处理 DMA 搬运完成的数据并更新滤波器。
 */
void  ADC_DMA_poll(void);

/**
 * @brief 获取最新滤波后的 8 通道 ADC 电压值
 *
 * 内部先调用 ADC_DMA_poll() 处理新数据,
 * 若有新数据则将累加和转换为浮点电压值。
 *
 * @return 指向 8 个 float 的数组 (单位: 伏特), 静态缓冲区
 */
const float* ADC_DMA_get_value(void);

/**
 * @brief 重置滤波器: 清空所有累加器和环形缓冲区
 *
 * 在通道切换或需要重新稳定时调用。
 */
void  ADC_DMA_filter_reset(void);

/**
 * @brief 检查滤波器是否已收集满一个完整窗口的数据
 * @return true 数据已稳定 (4个块全部填充)
 */
bool  ADC_DMA_ready(void);

/**
 * @brief 阻塞等待滤波器填满
 *
 * 最多等待 2 秒, 超时后返回 (即使数据未完全稳定)。
 * 用于初始化后的首次数据稳定。
 */
void  ADC_DMA_wait_full(void);
