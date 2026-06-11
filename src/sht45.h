/**
 * @file sht45.h
 * @brief SHT45 温湿度传感器驱动头文件
 *
 * SHT45 是 Sensirion 公司的高精度数字温湿度传感器。
 * 通信接口：I2C（地址 0x44）
 * 温度范围：-40°C ~ +125°C，精度 ±0.1°C
 * 湿度范围：0% ~ 100%RH，精度 ±1.0%RH
 * 数据帧格式：6 字节（温度高/低/ CRC + 湿度高/低/ CRC）
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 SHT45 传感器
 *
 * 内部调用 i2c2_init() 初始化 I2C2 硬件，
 * 然后等待 10ms 使传感器上电稳定。
 */
void sht45_init(void);

/**
 * @brief 读取温度值
 *
 * @param temperature  输出参数，温度值（单位：°C）
 * @return             0=读取成功，1=CRC校验失败或I2C通信错误
 */
uint8_t sht45_read_temperature(float *temperature);

/**
 * @brief 读取湿度值
 *
 * @param humidity  输出参数，相对湿度（单位：%RH，已钳位到 0~100）
 * @return          0=读取成功，1=CRC校验失败或I2C通信错误
 */
uint8_t sht45_read_humidity(float *humidity);

#ifdef __cplusplus
}
#endif
