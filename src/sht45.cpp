/**
 * @file sht45.cpp
 * @brief SHT45 温湿度传感器驱动实现
 *
 * SHT45 使用高精度测量命令（0xFD, 0x00），测量时间约 8.2ms。
 * 每次测量返回 6 字节数据：
 *   [温度MSB][温度LSB][温度CRC][湿度MSB][湿度LSB][湿度CRC]
 *
 * CRC-8 校验多项式：x^8 + x^5 + x^4 + 1 (0x31)，初始值 0xFF
 * 温度转换公式：T = -45 + 175 × raw / 65535
 * 湿度转换公式：RH = -6 + 125 × raw / 65535
 */

#include "sht45.h"
#include "i2c2.h"
#include "hal/time_hw.h"

/* SHT45 的 7 位 I2C 地址（硬件固定为 0x44） */
#define SHT45_ADDR 0x44

/**
 * @brief CRC-8 校验计算
 *
 * @param data  待校验的数据
 * @param len   数据长度
 * @return      CRC-8 校验值
 *
 * 使用多项式 0x31（x^8 + x^5 + x^4 + 1），初始值 0xFF。
 * 这是 Sensirion 传感器标准的 CRC-8 算法。
 */
static uint8_t sht45_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFF;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief 初始化 SHT45 传感器
 *
 * 先初始化 I2C2 硬件，再等待 10ms 让传感器上电稳定。
 */
void sht45_init(void)
{
    i2c2_init();
    delay(10);
}

/**
 * @brief 内部函数：执行一次测量并同时获取温度和湿度
 *
 * @param temperature  输出温度值（°C）
 * @param humidity     输出湿度值（%RH）
 * @return             0=成功，1=失败
 *
 * 测量流程：
 * 1. 发送高精度测量命令 (0xFD, 0x00)
 * 2. 等待 10ms（SHT45 高精度模式测量时间约 8.2ms）
 * 3. 读取 6 字节数据
 * 4. 分别校验温度和湿度的 CRC-8
 * 5. 将原始 16 位数据转换为物理量
 */
static uint8_t sht45_measure(float *temperature, float *humidity)
{
    /* 发送高精度测量命令 */
    uint8_t cmd[2] = {0xFD, 0x00};
    if (i2c2_write(SHT45_ADDR, cmd, 2))
        return 1;

    /* 等待测量完成（高精度模式约 8.2ms，留 10ms 余量） */
    delay(10);

    /* 读取 6 字节数据：温度(2) + CRC(1) + 湿度(2) + CRC(1) */
    uint8_t buf[6];
    if (i2c2_read(SHT45_ADDR, buf, 6))
        return 1;

    /* 校验温度数据的 CRC */
    if (sht45_crc8(buf, 2) != buf[2])
        return 1;
    /* 校验湿度数据的 CRC */
    if (sht45_crc8(buf + 3, 2) != buf[5])
        return 1;

    /* 从原始数据中提取 16 位值 */
    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];  /* 温度原始值 */
    uint16_t raw_h = ((uint16_t)buf[3] << 8) | buf[4];  /* 湿度原始值 */

    /* 物理量转换公式（SHT45 datasheet 提供） */
    *temperature = -45.0f + 175.0f * (float)raw_t / 65535.0f;
    *humidity    = -6.0f  + 125.0f * (float)raw_h / 65535.0f;

    /* 湿度钳位到有效范围 0%~100% */
    if (*humidity < 0.0f)   *humidity = 0.0f;
    if (*humidity > 100.0f) *humidity = 100.0f;

    return 0;
}

/**
 * @brief 读取温度值（公开接口）
 *
 * 内部调用 sht45_measure() 同时获取温度和湿度，
 * 丢弃湿度结果，仅返回温度。
 */
uint8_t sht45_read_temperature(float *temperature)
{
    float humi;
    return sht45_measure(temperature, &humi);
}

/**
 * @brief 读取湿度值（公开接口）
 *
 * 内部调用 sht45_measure() 同时获取温度和湿度，
 * 丢弃温度结果，仅返回湿度。
 */
uint8_t sht45_read_humidity(float *humidity)
{
    float temp;
    return sht45_measure(&temp, humidity);
}
