#include "hal/time_hw.h"
#include "ch32v20x.h"

/** @brief 每微秒的tick数(默认1，由time_hw_init()根据系统时钟计算更新) */
uint32_t time_hw_tpus = 1;

/** @brief 每毫秒的tick数(默认1000，由time_hw_init()根据系统时钟计算更新) */
uint32_t time_hw_tpms = 1000;

/**
 * @brief 读取64位原始SysTick计数值(无符号扩展)
 * @return 64位tick值
 * @details SysTick为24位计数器，通过软件扩展为64位:
 *          1. 先读高32位(hi1)
 *          2. 读低32位(lo)
 *          3. 再读高32位(hi2)
 *          4. 若hi1≠hi2说明读取过程中发生溢出，重新读取
 *          5. 组合为64位值: (hi1 << 32) | lo
 *          注意: 实际CH32V203的SysTick是24位，高32位寄存器始终为0，
 *          此函数为通用实现，确保在任何位宽的SysTick上都能正确工作
 */
static inline __attribute__((always_inline)) uint64_t ticks64_raw(void)
{
    uint32_t hi1, lo, hi2;
    do {
        hi1 = STK_CNTH;    // 读高32位
        lo  = STK_CNTL;    // 读低32位
        hi2 = STK_CNTH;    // 再读高32位验证
    } while (hi1 != hi2);  // 若不一致则重读(防止读取过程中溢出)
    return ((uint64_t)hi1 << 32) | (uint64_t)lo;
}

/**
 * @brief 初始化硬件定时器
 * @details 配置步骤:
 *          1. 关闭SysTick(CTL=0)，清除状态和计数值
 *          2. 设置重装载值为最大值(0xFFFFFFFF)，实现自由运行
 *          3. 启动SysTick: STRE=1(重装载), STCLK=HCLK/8(时钟分频), STE=1(使能)
 *          4. 计算每微秒和每毫秒的tick数:
 *             tpus = (SystemCoreClock / 8) / 1000000
 *             tpms = tpus * 1000
 *          CH32V203默认SystemCoreClock=72MHz，分频后9MHz → 9 ticks/us, 9000 ticks/ms
 */
void time_hw_init(void)
{
    // 关闭SysTick并清除所有状态
    STK_CTLR = 0;
    STK_SR   = 0;
    STK_CNTL = 0;
    STK_CNTH = 0;

    // 设置最大重装载值，实现32位自由运行计数器
    STK_CMPLR = 0xFFFFFFFFu;
    STK_CMPHR = 0xFFFFFFFFu;

    // 启动SysTick: bit3=STRE(重装载使能), bit0=STE(计数器使能)
    // 时钟源: HCLK/8 (由硬件默认配置，或通过STK_CTLR的bit2设置)
    STK_CTLR = (1u << 3) | (1u << 0);

    // 计算每微秒的tick数: (SystemCoreClock/8) / 1000000
    uint32_t tpus = (SystemCoreClock / 8u) / 1000000u;
    if (!tpus) tpus = 1u;   // 防止除零
    time_hw_tpus = tpus;

    // 计算每毫秒的tick数: tpus * 1000
    uint32_t tpms = tpus * 1000u;
    if (!tpms) tpms = 1u;   // 防止除零
    time_hw_tpms = tpms;
}

/**
 * @brief 获取每微秒的tick数
 * @return time_hw_tpus的值
 */
uint32_t time_hw_ticks_per_us(void) { return time_hw_tpus; }

/**
 * @brief 获取每毫秒的tick数
 * @return time_hw_tpms的值
 */
uint32_t time_hw_ticks_per_ms(void) { return time_hw_tpms; }

/**
 * @brief 获取64位全局tick计数
 * @return 64位tick值
 */
uint64_t time_ticks64(void)
{
    return ticks64_raw();
}

/**
 * @brief 获取64位微秒时间戳
 * @return 自启动以来的微秒数
 * @details tick数除以每微秒tick数得到微秒值
 *          使用__builtin_expect优化分支预测(tpus=1时直接返回tick值)
 */
uint64_t time_us64(void)
{
    const uint64_t t = ticks64_raw();
    const uint32_t d = time_hw_tpus;
    if (__builtin_expect(d == 1u, 0)) return t;  // 若每us=1tick，直接返回(无除法开销)
    return t / (uint64_t)d;
}

/**
 * @brief 获取64位毫秒时间戳
 * @return 自启动以来的毫秒数
 * @details tick数除以每毫秒tick数得到毫秒值
 */
uint64_t time_ms64(void)
{
    const uint64_t t = ticks64_raw();
    const uint32_t d = time_hw_tpms;
    if (__builtin_expect(d == 1u, 0)) return t;
    return t / (uint64_t)d;
}

/**
 * @brief 微秒级延时
 * @param us 延时微秒数
 * @details 将微秒转换为tick数(溢出保护)，然后调用delayTicks32()忙等待
 */
void delay_us(uint32_t us)
{
    if (!us) return;
    uint64_t t = (uint64_t)us * (uint64_t)time_hw_tpus;
    if (t > 0xFFFFFFFFu) t = 0xFFFFFFFFu;  // 溢出保护
    delayTicks32((uint32_t)t);
}

/**
 * @brief 毫秒级延时
 * @param ms 延时毫秒数
 * @details 将毫秒转换为tick数(溢出保护)，然后调用delayTicks32()忙等待
 */
void delay(uint32_t ms)
{
    if (!ms) return;
    uint64_t t = (uint64_t)ms * (uint64_t)time_hw_tpms;
    if (t > 0xFFFFFFFFu) t = 0xFFFFFFFFu;  // 溢出保护
    delayTicks32((uint32_t)t);
}
