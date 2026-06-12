/**
 * @file i2c2.h
 * @brief 硬件 I2C2 驱动头文件
 *
 * 本模块提供 CH32V203C8T6 芯片 I2C2 外设的底层驱动接口。
 * I2C2 使用 PB10(SCL) 和 PB11(SDA) 引脚，支持 7 位地址模式。
 * 主要用于与 SHT45 温湿度传感器通信。
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 I2C2 外设
 *
 * 配置 GPIOB Pin10(SCL)/Pin11(SDA) 为复用开漏输出，
 * 设置 I2C2 工作在主机模式，时钟速率 100kHz。
 */
void i2c2_init(void);

/**
 * @brief I2C 主机发送数据
 *
 * @param addr7   7 位从机地址（不含读写位）
 * @param data    待发送的数据缓冲区
 * @param len     待发送的字节数
 * @return        0=成功，1=超时/总线错误
 *
 * 执行完整的 I2C 写时序：START → 地址+W → 数据 → STOP
 */
uint8_t i2c2_write(uint8_t addr7, const uint8_t *data, uint32_t len);

/**
 * @brief I2C 主机接收数据
 *
 * @param addr7   7 位从机地址（不含读写位）
 * @param buf     接收数据的缓冲区
 * @param len     期望接收的字节数
 * @return        0=成功，1=超时/总线错误
 *
 * 执行完整的 I2C 读时序：START → 地址+R → 接收数据(NACK最后1字节) → STOP
 */
uint8_t i2c2_read(uint8_t addr7, uint8_t *buf, uint32_t len);

/**
 * @brief I2C 主机写数据后重复 START 读数据（组合事务）
 *
 * @param addr7   7 位从机地址（不含读写位）
 * @param wdata   待写入的数据缓冲区
 * @param wlen    待写入的字节数
 * @param rbuf    接收数据的缓冲区
 * @param rlen    期望接收的字节数
 * @return        0=成功，1=超时/NACK/总线错误
 *
 * 执行 I2C 组合时序：START → 地址+W → 写数据 → Sr(重复START) → 地址+R → 读数据(NACK最后1字节) → STOP
 * 用于 BME280 寄存器读取等需要连续写-读事务的传感器。
 */
uint8_t i2c2_write_read(uint8_t addr7, const uint8_t *wdata, uint32_t wlen, uint8_t *rbuf, uint32_t rlen);

#ifdef __cplusplus
}
#endif
