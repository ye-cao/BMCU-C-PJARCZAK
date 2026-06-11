/**
 * @file crc_bus.h
 * @brief 总线通信 CRC 校验头文件
 *
 * 提供 BambuBus/AHUB 总线通信所需的 CRC-8 和 CRC-16 校验函数。
 * 使用查表法实现，适用于嵌入式系统的高效 CRC 计算。
 */

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 CRC 模块（当前为空操作）
 *
 * CRC 查表法无需运行时初始化，保留此接口以保持架构一致性。
 */
void bus_crc_init(void);

/**
 * @brief 计算 CRC-8 校验值
 *
 * @param data  待校验数据
 * @param len   数据长度（字节）
 * @return      CRC-8 校验值
 *
 * 初始值：0x66
 * 使用预计算查找表加速，每个字节通过一次查表和异或运算处理。
 */
uint8_t bus_crc8(const uint8_t* data, uint32_t len);

/**
 * @brief 计算 CRC-16 校验值
 *
 * @param data  待校验数据
 * @param len   数据长度（字节）
 * @return      CRC-16 校验值
 *
 * 初始值：0x913D
 * 使用预计算查找表加速，每次取高 8 位异或数据字节作为查表索引。
 */
uint16_t bus_crc16(const uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif
