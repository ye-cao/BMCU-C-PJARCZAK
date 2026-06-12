/**
 * @file th_sensor.h
 * @brief 通用 I2C 温湿度传感器驱动头文件
 *
 * 支持市面上常见的 I2C 温湿度传感器，自动检测连接的传感器类型：
 *   - SHT40/41/43/45（Sensirion，地址 0x44）
 *   - SHT30/31/35（Sensirion，地址 0x44）
 *   - AHT20/AHT21（奥松，地址 0x38）
 *   - Si7021/HTU21D（Silicon Labs，地址 0x40）
 *   - HDC1080（TI，地址 0x40）
 *   - BME280（Bosch，地址 0x76/0x77，温+湿+气压）
 *
 * 使用方法：
 *   1. 调用 th_sensor_init() 自动检测并初始化传感器
 *   2. 调用 th_sensor_read() 读取温湿度数据
 *   3. 调用 th_sensor_name() 获取传感器型号名称
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化温湿度传感器（自动检测）
 *
 * 初始化 I2C2 总线，然后依次尝试检测以下传感器：
 * AHT20 → Si7021 → HDC1080 → SHT4x → SHT3x → BME280
 * 检测到的第一个有效传感器将被记录，后续读取使用该传感器。
 */
void th_sensor_init(void);

/**
 * @brief 读取温湿度数据
 *
 * @param temperature  输出参数，温度值（单位：°C）
 * @param humidity     输出参数，相对湿度（单位：%RH，已钳位到 0~100）
 * @return             0=读取成功，1=无传感器或读取失败
 */
uint8_t th_sensor_read(float *temperature, float *humidity);

/**
 * @brief 获取检测到的传感器名称
 *
 * @return  传感器型号字符串（如 "SHT45"、"AHT20" 等），
 *          未检测到时返回 "NONE"
 */
const char* th_sensor_name(void);

#ifdef __cplusplus
}
#endif
