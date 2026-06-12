/**
 * @file th_sensor.cpp
 * @brief 通用 I2C 温湿度传感器驱动实现
 *
 * 支持 6 大类传感器的自动检测和读取：
 *   - SHT40/41/43/45（Sensirion，高精度，地址 0x44）
 *   - SHT30/31/35（Sensirion，地址 0x44）
 *   - AHT20/AHT21（奥松，最便宜，地址 0x38）
 *   - Si7021/HTU21D（Silicon Labs，地址 0x40）
 *   - HDC1080（TI，地址 0x40）
 *   - BME280（Bosch，温+湿+气压，地址 0x76/0x77）
 *
 * 自动检测流程（按地址扫描）：
 *   1. 0x38 → AHT20/AHT21
 *   2. 0x40 → Si7021/HTU21D（优先）或 HDC1080
 *   3. 0x44 → SHT4x（优先）或 SHT3x
 *   4. 0x76/0x77 → BME280
 */

#include "th_sensor.h"
#include "i2c2.h"
#include "hal/time_hw.h"

/* ========== 传感器 I2C 地址定义 ========== */
#define ADDR_AHT20      0x38
#define ADDR_Si7021     0x40
#define ADDR_SHT        0x44
#define ADDR_BME280_LO  0x76
#define ADDR_BME280_HI  0x77

/* ========== 传感器类型枚举 ========== */
typedef enum {
    TH_NONE = 0,
    TH_SHT4x,
    TH_SHT3x,
    TH_AHT20,
    TH_SI7021,
    TH_HDC1080,
    TH_BME280
} th_type_t;

static th_type_t detected_type = TH_NONE;

/* ========== 通用 CRC-8（Sensirion / AHT20） ========== */
static uint8_t crc8_sensirion(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFF;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ================================================================== */
/*                        SHT40/41/43/45 驱动                         */
/* ================================================================== */

static uint8_t try_sht4x(void)
{
    uint8_t cmd[2] = {0xFD, 0x00};
    if (i2c2_write(ADDR_SHT, cmd, 2))
        return 0;
    delay(10);
    uint8_t buf[6];
    if (i2c2_read(ADDR_SHT, buf, 6))
        return 0;
    if (crc8_sensirion(buf, 2) != buf[2])
        return 0;
    if (crc8_sensirion(buf + 3, 2) != buf[5])
        return 0;
    return 1;
}

static uint8_t read_sht4x(float *t, float *h)
{
    uint8_t cmd[2] = {0xFD, 0x00};
    if (i2c2_write(ADDR_SHT, cmd, 2))
        return 1;
    delay(10);
    uint8_t buf[6];
    if (i2c2_read(ADDR_SHT, buf, 6))
        return 1;
    if (crc8_sensirion(buf, 2) != buf[2])
        return 1;
    if (crc8_sensirion(buf + 3, 2) != buf[5])
        return 1;
    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_h = ((uint16_t)buf[3] << 8) | buf[4];
    *t = -45.0f + 175.0f * (float)raw_t / 65535.0f;
    *h = -6.0f + 125.0f * (float)raw_h / 65535.0f;
    if (*h < 0.0f)   *h = 0.0f;
    if (*h > 100.0f) *h = 100.0f;
    return 0;
}

/* ================================================================== */
/*                        SHT30/31/35 驱动                            */
/* ================================================================== */

static uint8_t try_sht3x(void)
{
    uint8_t cmd[2] = {0x24, 0x00};
    if (i2c2_write(ADDR_SHT, cmd, 2))
        return 0;
    delay(15);
    uint8_t buf[6];
    if (i2c2_read(ADDR_SHT, buf, 6))
        return 0;
    if (crc8_sensirion(buf, 2) != buf[2])
        return 0;
    if (crc8_sensirion(buf + 3, 2) != buf[5])
        return 0;
    return 1;
}

static uint8_t read_sht3x(float *t, float *h)
{
    uint8_t cmd[2] = {0x24, 0x00};
    if (i2c2_write(ADDR_SHT, cmd, 2))
        return 1;
    delay(15);
    uint8_t buf[6];
    if (i2c2_read(ADDR_SHT, buf, 6))
        return 1;
    if (crc8_sensirion(buf, 2) != buf[2])
        return 1;
    if (crc8_sensirion(buf + 3, 2) != buf[5])
        return 1;
    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_h = ((uint16_t)buf[3] << 8) | buf[4];
    *t = -45.0f + 175.0f * (float)raw_t / 65535.0f;
    *h = 100.0f * (float)raw_h / 65535.0f;
    if (*h < 0.0f)   *h = 0.0f;
    if (*h > 100.0f) *h = 100.0f;
    return 0;
}

/* ================================================================== */
/*                        AHT20/AHT21 驱动                            */
/* ================================================================== */

static uint8_t try_aht20(void)
{
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    if (i2c2_write(ADDR_AHT20, cmd, 3))
        return 0;
    delay(80);
    uint8_t buf[7];
    if (i2c2_read(ADDR_AHT20, buf, 7))
        return 0;
    if (buf[0] & 0x80)
        return 0;
    if (!(buf[0] & 0x08))
        return 0;
    if (crc8_sensirion(buf, 6) != buf[6])
        return 0;
    return 1;
}

static uint8_t read_aht20(float *t, float *h)
{
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    if (i2c2_write(ADDR_AHT20, cmd, 3))
        return 1;
    delay(80);
    uint8_t buf[7];
    if (i2c2_read(ADDR_AHT20, buf, 7))
        return 1;
    if (buf[0] & 0x80)
        return 1;
    if (crc8_sensirion(buf, 6) != buf[6])
        return 1;
    uint32_t raw_h = ((uint32_t)buf[1] << 12) |
                     ((uint32_t)buf[2] << 4)  |
                     ((uint32_t)buf[3] >> 4);
    uint32_t raw_t = ((uint32_t)(buf[3] & 0x0F) << 16) |
                     ((uint32_t)buf[4] << 8) |
                     ((uint32_t)buf[5]);
    *h = 100.0f * (float)raw_h / 1048576.0f;
    *t = 200.0f * (float)raw_t / 1048576.0f - 50.0f;
    if (*h < 0.0f)   *h = 0.0f;
    if (*h > 100.0f) *h = 100.0f;
    return 0;
}

/* ================================================================== */
/*                        Si7021/HTU21D 驱动                          */
/* ================================================================== */

static uint8_t try_si7021(void)
{
    uint8_t cmd = 0xE3;
    if (i2c2_write(ADDR_Si7021, &cmd, 1))
        return 0;
    delay(15);
    uint8_t buf[3];
    if (i2c2_read(ADDR_Si7021, buf, 3))
        return 0;
    if (crc8_sensirion(buf, 2) != buf[2])
        return 0;
    return 1;
}

static uint8_t read_si7021(float *t, float *h)
{
    uint8_t cmd = 0xE5;
    if (i2c2_write(ADDR_Si7021, &cmd, 1))
        return 1;
    delay(15);
    uint8_t buf[3];
    if (i2c2_read(ADDR_Si7021, buf, 3))
        return 1;
    if (crc8_sensirion(buf, 2) != buf[2])
        return 1;
    uint16_t raw_h = ((uint16_t)buf[0] << 8) | buf[1];
    *h = -6.0f + 125.0f * (float)raw_h / 65536.0f;
    if (*h < 0.0f)   *h = 0.0f;
    if (*h > 100.0f) *h = 100.0f;

    cmd = 0xE0;
    if (i2c2_write(ADDR_Si7021, &cmd, 1))
        return 0;
    delay(10);
    if (i2c2_read(ADDR_Si7021, buf, 3))
        return 0;
    if (crc8_sensirion(buf, 2) != buf[2])
        return 0;
    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
    *t = -46.85f + 175.72f * (float)raw_t / 65536.0f;
    return 0;
}

/* ================================================================== */
/*                          HDC1080 驱动                              */
/* ================================================================== */

static uint8_t try_hdc1080(void)
{
    uint8_t cmd = 0xFE;
    if (i2c2_write(ADDR_Si7021, &cmd, 1))
        return 0;
    delay(5);
    uint8_t buf[2];
    if (i2c2_read(ADDR_Si7021, buf, 2))
        return 0;
    uint16_t mfg_id = ((uint16_t)buf[0] << 8) | buf[1];
    if (mfg_id != 0x5449)
        return 0;
    return 1;
}

static uint8_t read_hdc1080(float *t, float *h)
{
    uint8_t cmd = 0x00;
    if (i2c2_write(ADDR_Si7021, &cmd, 1))
        return 1;
    delay(10);
    uint8_t buf[4];
    if (i2c2_read(ADDR_Si7021, buf, 4))
        return 1;
    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_h = ((uint16_t)buf[2] << 8) | buf[3];
    *t = 165.0f * (float)raw_t / 65536.0f - 40.0f;
    *h = 100.0f * (float)raw_h / 65536.0f;
    if (*h < 0.0f)   *h = 0.0f;
    if (*h > 100.0f) *h = 100.0f;
    return 0;
}

/* ================================================================== */
/*                          BME280 驱动                               */
/* ================================================================== */

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_cal_t;

static bme280_cal_t bme_cal;
static uint8_t bme_addr = 0x76;

static uint8_t bme280_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
    return i2c2_write_read(bme_addr, &reg, 1, buf, len);
}

static uint8_t bme280_read_reg_at(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c2_write_read(addr, &reg, 1, val, 1);
}

static uint8_t bme280_read_calibration(void)
{
    uint8_t buf[26];
    if (bme280_read_reg(0x88, buf, 26))
        return 1;
    bme_cal.dig_T1 = (uint16_t)((uint16_t)buf[1] << 8 | buf[0]);
    bme_cal.dig_T2 = (int16_t)((int16_t)buf[3] << 8 | buf[2]);
    bme_cal.dig_T3 = (int16_t)((int16_t)buf[5] << 8 | buf[4]);
    bme_cal.dig_H1 = buf[25];
    if (bme280_read_reg(0xE1, buf, 7))
        return 1;
    bme_cal.dig_H2 = (int16_t)((int16_t)buf[1] << 8 | buf[0]);
    bme_cal.dig_H3 = buf[2];
    bme_cal.dig_H4 = (int16_t)((int16_t)buf[3] << 4 | (buf[4] & 0x0F));
    bme_cal.dig_H5 = (int16_t)((int16_t)buf[5] << 4 | ((buf[4] >> 4) & 0x0F));
    bme_cal.dig_H6 = (int8_t)buf[6];
    return 0;
}

static int32_t bme280_compensate_T(int32_t adc_T, int32_t *t_fine)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)bme_cal.dig_T1 << 1))) *
            ((int32_t)bme_cal.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bme_cal.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)bme_cal.dig_T1))) >> 12) *
            ((int32_t)bme_cal.dig_T3)) >> 14;
    *t_fine = var1 + var2;
    T = (*t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t bme280_compensate_H(int32_t adc_H, int32_t t_fine)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    if (v_x1_u32r == 0)
        return 0;
    v_x1_u32r = (((adc_H << 14) - ((int32_t)bme_cal.dig_H4 << 20) -
                  ((int32_t)bme_cal.dig_H5 * v_x1_u32r) + 16384) >> 15);
    v_x1_u32r = (((((((v_x1_u32r * v_x1_u32r) >> 7) *
                      ((int32_t)bme_cal.dig_H3)) >> 2) +
                    ((((v_x1_u32r * bme_cal.dig_H3) >> 1))) * 32768) >> 16));
    v_x1_u32r = (v_x1_u32r * (((((int32_t)bme_cal.dig_H2 * v_x1_u32r) >> 15) +
                  (((int32_t)bme_cal.dig_H1 * v_x1_u32r) >> 12)) >> 10)) >> 11;
    if (v_x1_u32r > 419430400)
        v_x1_u32r = 419430400;
    return (uint32_t)(v_x1_u32r >> 12);
}

static uint8_t try_bme280(void)
{
    uint8_t addrs[2] = {ADDR_BME280_LO, ADDR_BME280_HI};
    for (uint8_t i = 0; i < 2; i++)
    {
        uint8_t chip_id;
        if (bme280_read_reg_at(addrs[i], 0xD0, &chip_id))
            continue;
        if (chip_id != 0x60)
            continue;
        bme_addr = addrs[i];
        if (bme280_read_calibration())
            return 0;
        uint8_t ctrl_hum = 0x01;
        uint8_t reg = 0xF2;
        if (i2c2_write(bme_addr, &reg, 1))
            continue;
        if (i2c2_write(bme_addr, &ctrl_hum, 1))
            continue;
        uint8_t ctrl_meas = 0x27;
        reg = 0xF4;
        if (i2c2_write(bme_addr, &reg, 1))
            continue;
        if (i2c2_write(bme_addr, &ctrl_meas, 1))
            continue;
        return 1;
    }
    return 0;
}

static uint8_t read_bme280(float *t, float *h)
{
    uint8_t buf[8];
    if (bme280_read_reg(0xF7, buf, 8))
        return 1;
    int32_t adc_T = (int32_t)(((uint32_t)buf[3] << 12) |
                              ((uint32_t)buf[4] << 4)  |
                              ((uint32_t)buf[5] >> 4));
    int32_t adc_H = (int32_t)(((uint32_t)buf[6] << 8) | (uint32_t)buf[7]);
    int32_t t_fine;
    int32_t temp_x100 = bme280_compensate_T(adc_T, &t_fine);
    uint32_t hum_x1024 = bme280_compensate_H(adc_H, t_fine);
    *t = (float)temp_x100 / 100.0f;
    *h = (float)hum_x1024 / 1024.0f;
    if (*h < 0.0f)   *h = 0.0f;
    if (*h > 100.0f) *h = 100.0f;
    return 0;
}

/* ================================================================== */
/*                         公共 API 实现                               */
/* ================================================================== */

void th_sensor_init(void)
{
    i2c2_init();
    delay(10);
    detected_type = TH_NONE;

    if (try_aht20())   { detected_type = TH_AHT20;   return; }
    if (try_si7021())  { detected_type = TH_SI7021;  return; }
    if (try_hdc1080()) { detected_type = TH_HDC1080; return; }
    if (try_sht4x())   { detected_type = TH_SHT4x;   return; }
    if (try_sht3x())   { detected_type = TH_SHT3x;   return; }
    if (try_bme280())  { detected_type = TH_BME280;  return; }
}

uint8_t th_sensor_read(float *temperature, float *humidity)
{
    switch (detected_type)
    {
        case TH_SHT4x:   return read_sht4x(temperature, humidity);
        case TH_SHT3x:   return read_sht3x(temperature, humidity);
        case TH_AHT20:   return read_aht20(temperature, humidity);
        case TH_SI7021:  return read_si7021(temperature, humidity);
        case TH_HDC1080: return read_hdc1080(temperature, humidity);
        case TH_BME280:  return read_bme280(temperature, humidity);
        default:         return 1;
    }
}

const char* th_sensor_name(void)
{
    switch (detected_type)
    {
        case TH_SHT4x:   return "SHT4x";
        case TH_SHT3x:   return "SHT3x";
        case TH_AHT20:   return "AHT20";
        case TH_SI7021:  return "Si7021";
        case TH_HDC1080: return "HDC1080";
        case TH_BME280:  return "BME280";
        default:         return "NONE";
    }
}
