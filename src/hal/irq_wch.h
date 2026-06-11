#pragma once
#include <stdint.h>

/**
 * @file irq_wch.h
 * @brief WCH RISC-V中断控制工具头文件
 * @details 提供WCH CH32V系列RISC-V MCU的中断保存/恢复内联函数。
 *          通过操作mstatus寄存器(地址0x800)的MIE和MPIE位实现临界区保护。
 *          WCH的ECLIC中断控制器使用自定义mstatus扩展位。
 */

/**
 * @brief 保存当前中断状态并关闭全局中断
 * @return 之前的中断状态位(仅保留bit3(MIE)和bit6(MPIE))
 * @details RISC-V内联汇编:
 *          1. csrr %0, 0x800 → 读取mstatus寄存器到变量s
 *          2. csrc 0x800, 0x88 → 清除mstatus的bit3(MIE)和bit6(MPIE)
 *             0x88 = 0b10001000，即bit3和bit6
 *          3. 返回原始状态中bit3和bit6的值(用于后续恢复)
 *          这是进入临界区的标准模式: 先保存状态，再关中断
 */
static inline __attribute__((always_inline)) uint32_t irq_save_wch(void)
{
    uint32_t s;
    __asm volatile("csrr %0, 0x800" : "=r"(s) :: "memory");  // 读取mstatus
    __asm volatile("csrc 0x800, %0" :: "r"(0x88u) : "memory");  // 清除MIE和MPIE位
    return (s & 0x88u);  // 返回需要恢复的位
}

/**
 * @brief 恢复之前保存的中断状态
 * @param s88 之前irq_save_wch()返回的状态值(仅bit3和bit6有效)
 * @details RISC-V内联汇编:
 *          1. csrc 0x800, 0x88 → 先清除MIE和MPIE位(确保原子操作)
 *          2. csrs 0x800, s88 → 再设置之前保存的位(恢复原始状态)
 *          这是退出临界区的标准模式: 恢复之前保存的中断状态
 */
static inline __attribute__((always_inline)) void irq_restore_wch(uint32_t s88)
{
    const uint32_t m = 0x88u;
    __asm volatile("csrc 0x800, %0" :: "r"(m)   : "memory");  // 先清除MIE/MPIE
    __asm volatile("csrs 0x800, %0" :: "r"(s88) : "memory");  // 再恢复原始状态
}
