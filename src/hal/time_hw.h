#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file time_hw.h
 * @brief 硬件定时器抽象层头文件
 * @details 基于CH32V203 SysTick定时器提供毫秒/微秒级的时间管理和延时功能。
 *          SysTick为24位递减计数器，本驱动通过32位/64位软件计数器扩展为无溢出的全局时基。
 */

/** @brief SysTick寄存器组基地址(CH32V203存储器映射: 0xE000F000) */
#define TIME_HW_STK_BASE (0xE000F000u)

/** @brief SysTick控制寄存器(偏移0x00) - 使能、时钟源、中断等控制位 */
#define STK_CTLR  (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x00u))

/** @brief SysTick状态寄存器(偏移0x04) - 计数溢出标志位 */
#define STK_SR    (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x04u))

/** @brief SysTick当前值寄存器低32位(偏移0x08) - 读取当前计数值 */
#define STK_CNTL  (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x08u))

/** @brief SysTick当前值寄存器高32位(偏移0x0C) - 用于64位扩展计数 */
#define STK_CNTH  (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x0Cu))

/** @brief SysTick重装载值寄存器低32位(偏移0x10) - 设置计数周期(0xFFFFFFFF=最大) */
#define STK_CMPLR (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x10u))

/** @brief SysTick重装载值寄存器高32位(偏移0x14) - 用于64位扩展 */
#define STK_CMPHR (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x14u))

/**
 * @brief 初始化硬件定时器(SysTick)
 * @details 配置SysTick为自由运行模式(不产生中断)，使用HCLK/8作为时钟源，
 *          计算每微秒和每毫秒对应的tick数
 */
void time_hw_init(void);

/**
 * @brief 获取每微秒对应的tick数
 * @return 每微秒的tick数(如72MHz/8=9 → 9 ticks/us)
 */
uint32_t time_hw_ticks_per_us(void);

/**
 * @brief 获取每毫秒对应的tick数
 * @return 每毫秒的tick数(如9000 ticks/ms)
 */
uint32_t time_hw_ticks_per_ms(void);

/** @brief 每微秒的tick数(全局变量，由time_hw_init()计算) */
extern uint32_t time_hw_tpus;

/** @brief 每毫秒的tick数(全局变量，由time_hw_init()计算) */
extern uint32_t time_hw_tpms;

/**
 * @brief 计算两个32位无符号时间戳的差值(a - b)
 * @param a 较大的时间戳(或当前时间)
 * @param b 较小的时间戳(或起始时间)
 * @return 无符号差值(自动处理溢出)
 * @details 利用无符号整数减法的自然回绕特性，即使a<b也能得到正确差值
 */
static inline __attribute__((always_inline)) uint32_t time_diff_u32(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b);
}

/**
 * @brief 判断当前时间是否已到达或超过截止时间
 * @param now      当前时间戳
 * @param deadline 截止时间戳
 * @return 1=已到达/超过，0=未到达
 * @details 通过检查差值的最高位(符号位)判断:
 *          若(now - deadline)的bit31=0，说明now>=deadline(已到达)
 */
static inline __attribute__((always_inline)) int time_reached32(uint32_t now, uint32_t deadline)
{
    return ((time_diff_u32(now, deadline) >> 31) == 0u);
}

/**
 * @brief 计算两个32位时间戳的有符号差值(a - b)
 * @param a 时间戳a
 * @param b 时间戳b
 * @return 有符号差值(可判断时间先后关系)
 * @details 返回值>0表示a在b之后，<0表示a在b之前
 */
static inline __attribute__((always_inline)) int32_t time_diff32(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b);
}

/**
 * @brief 将毫秒数转换为tick数
 * @param ms 毫秒数
 * @return 对应的tick数，若参数无效或溢出则返回0或0xFFFFFFFF
 * @details 乘法溢出保护: 若ms > 0xFFFFFFFF/tpm，返回最大值0xFFFFFFFF
 */
static inline __attribute__((always_inline)) uint32_t ms_to_ticks32(uint32_t ms)
{
    const uint32_t tpm = time_hw_tpms;
    if (!ms || !tpm) return 0u;

    const uint32_t max_ms = 0xFFFFFFFFu / tpm;
    if (ms > max_ms) return 0xFFFFFFFFu;

    return ms * tpm;
}

/**
 * @brief 将微秒数转换为tick数
 * @param us 微秒数
 * @return 对应的tick数，若参数无效或溢出则返回0或0xFFFFFFFF
 * @details 乘法溢出保护: 若us > 0xFFFFFFFF/tpu，返回最大值0xFFFFFFFF
 */
static inline __attribute__((always_inline)) uint32_t us_to_ticks32(uint32_t us)
{
    const uint32_t tpu = time_hw_tpus;
    if (!us || !tpu) return 0u;

    const uint32_t max_us = 0xFFFFFFFFu / tpu;
    if (us > max_us) return 0xFFFFFFFFu;

    return us * tpu;
}

/**
 * @brief 获取当前32位SysTick计数值(自由运行，不溢出处理)
 * @return 当前计数值(24位有效，高位补零)
 * @details 直接读取STK_CNTL寄存器，适用于短时间间隔测量
 */
static inline __attribute__((always_inline)) uint32_t time_ticks32(void)
{
    return STK_CNTL;
}

/**
 * @brief 获取64位全局tick计数(扩展SysTick为64位)
 * @return 64位tick值(不会溢出，可用很长时间)
 */
uint64_t time_ticks64(void);

/**
 * @brief 获取64位微秒时间戳
 * @return 自系统启动以来的微秒数
 */
uint64_t time_us64(void);

/**
 * @brief 获取64位毫秒时间戳
 * @return 自系统启动以来的毫秒数
 */
uint64_t time_ms64(void);

/**
 * @brief 忙等待指定tick数的延时
 * @param ticks 需要等待的tick数
 * @details 通过读取STK_CNTL寄存器轮询等待，零开销(无中断/上下文切换)
 *          适用于极短延时(如I2C时序，通常几us)
 */
static inline __attribute__((always_inline)) void delayTicks32(uint32_t ticks)
{
    if (!ticks) return;
    const uint32_t t0 = STK_CNTL;
    while ((uint32_t)(STK_CNTL - t0) < ticks) {
        __asm__ volatile ("nop");   // 空指令，避免编译器优化
    }
}

/**
 * @brief 微秒级延时
 * @param us 延时微秒数
 * @details 将微秒转换为tick数后调用delayTicks32()
 */
void delay_us(uint32_t us);

/**
 * @brief 毫秒级延时
 * @param ms 延时毫秒数
 * @details 将毫秒转换为tick数后调用delayTicks32()
 */
void delay(uint32_t ms);

#ifdef __cplusplus
}
#endif
