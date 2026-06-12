/**
 * @file i2c2.cpp
 * @brief 硬件 I2C2 驱动实现
 *
 * 本模块实现 CH32V203C8T6 的 I2C2 硬件外设驱动。
 * 引脚分配：PB10=I2C2_SCL, PB11=I2C2_SDA
 * 通信速率：100kHz 标准模式
 * 用途：与 SHT45 温湿度传感器通信
 */

#include "i2c2.h"
#include "ch32v20x.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_i2c.h"
#include "hal/time_hw.h"

/**
 * @brief 等待 I2C2 事件标志
 *
 * @param flag       期望的事件标志（如 I2C_EVENT_MASTER_MODE_SELECT）
 * @param timeout_ms 超时时间（毫秒）
 *
 * 轮询等待指定的 I2C 事件，超时后直接跳出。
 * 注意：超时后不会返回错误，调用方需自行检查后续操作是否成功。
 */
static void i2c2_wait_flag(uint32_t flag, uint32_t timeout_ms)
{
    uint32_t t0 = time_ms64();
    while (!I2C_CheckEvent(I2C2, flag))
    {
        if ((time_ms64() - t0) > timeout_ms) break;
    }
}

/**
 * @brief I2C2 总线复位
 *
 * 当总线出现异常（如从机未释放 SDA 导致的挂死）时，
 * 通过软件复位 I2C2 外设来恢复总线状态。
 * 操作顺序：禁用 → 软件复位 → 清除复位 → 重新启用
 */
static void i2c2_bus_reset(void)
{
    I2C_Cmd(I2C2, DISABLE);
    I2C_SoftwareResetCmd(I2C2, ENABLE);
    I2C_SoftwareResetCmd(I2C2, DISABLE);
    I2C_Cmd(I2C2, ENABLE);
}

/**
 * @brief 初始化 I2C2 外设
 *
 * 完成以下配置：
 * 1. 使能 GPIOB 和 I2C2 时钟
 * 2. 配置 PB10(SCL) 和 PB11(SDA) 为 50MHz 复用开漏输出
 * 3. 设置 I2C2 为：
 *    - I2C 模式（非 SMBUS）
 *    - 占空比 2:1（标准模式）
 *    - 7 位从机地址
 *    - 时钟速率 100kHz
 * 4. 启用 I2C2 外设
 */
void i2c2_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    I2C_InitTypeDef  i2c  = {0};

    /* 使能 GPIOB 和 I2C2 外设时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    /* 配置 PB10 为复用开漏输出（SCL） */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &gpio);

    /* 配置 PB11 为复用开漏输出（SDA） */
    gpio.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &gpio);

    /* 配置 I2C2 参数 */
    i2c.I2C_Mode                = I2C_Mode_I2C;         /* I2C 模式 */
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;       /* 占空比 2:1 */
    i2c.I2C_OwnAddress1         = 0x00;                  /* 本机地址（主机模式不使用） */
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  /* 7 位地址 */
    i2c.I2C_ClockSpeed          = 100000;                /* 100kHz 标准模式 */

    I2C_Init(I2C2, &i2c);
    I2C_Cmd(I2C2, ENABLE);
}

/**
 * @brief I2C 主机写数据
 *
 * @param addr7   7 位从机地址（如 SHT45 的 0x44）
 * @param data    指向待发送数据的指针
 * @param len     要发送的字节数
 * @return        0=发送成功，1=总线忙超时
 *
 * I2C 写时序：
 * 1. 等待总线空闲
 * 2. 发送 START
 * 3. 发送从机地址 + 写位（W=0）
 * 4. 逐字节发送数据，每次发送后等待从机 ACK
 * 5. 发送 STOP 结束通信
 */
uint8_t i2c2_write(uint8_t addr7, const uint8_t *data, uint32_t len)
{
    uint32_t timeout_ms = 50;

    /* 等待总线空闲 */
    uint32_t t0 = time_ms64();
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        if ((time_ms64() - t0) > timeout_ms) { i2c2_bus_reset(); return 1; }
    }

    /* 发送起始信号 */
    I2C_GenerateSTART(I2C2, ENABLE);
    i2c2_wait_flag(I2C_EVENT_MASTER_MODE_SELECT, timeout_ms);

    /* 发送从机地址（7位地址左移1位 + 写位0） */
    I2C_Send7bitAddress(I2C2, addr7 << 1, I2C_Direction_Transmitter);
    i2c2_wait_flag(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout_ms);

    /* 检查从机是否应答（NACK = 从机不存在） */
    if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF))
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_AF);
        return 1;
    }

    /* 逐字节发送数据 */
    for (uint32_t i = 0; i < len; i++)
    {
        I2C_SendData(I2C2, data[i]);
        i2c2_wait_flag(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout_ms);
    }

    /* 发送停止信号 */
    I2C_GenerateSTOP(I2C2, ENABLE);
    return 0;
}

/**
 * @brief I2C 主机读数据
 *
 * @param addr7   7 位从机地址（如 SHT45 的 0x44）
 * @param buf     指向接收缓冲区的指针
 * @param len     期望接收的字节数
 * @return        0=读取成功，1=总线忙超时
 *
 * I2C 读时序：
 * 1. 等待总线空闲
 * 2. 使能 ACK（默认应答）
 * 3. 发送 START
 * 4. 发送从机地址 + 读位（R=1）
 * 5. 逐字节接收数据，最后 1 字节发送 NACK 表示结束
 * 6. 发送 STOP 结束通信
 */
uint8_t i2c2_read(uint8_t addr7, uint8_t *buf, uint32_t len)
{
    uint32_t timeout_ms = 50;

    /* 等待总线空闲 */
    uint32_t t0 = time_ms64();
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        if ((time_ms64() - t0) > timeout_ms) { i2c2_bus_reset(); return 1; }
    }

    /* 使能应答（默认） */
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    /* 发送起始信号 */
    I2C_GenerateSTART(I2C2, ENABLE);
    i2c2_wait_flag(I2C_EVENT_MASTER_MODE_SELECT, timeout_ms);

    /* 发送从机地址（7位地址左移1位 + 读位1） */
    I2C_Send7bitAddress(I2C2, addr7 << 1, I2C_Direction_Receiver);
    i2c2_wait_flag(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, timeout_ms);

    /* 检查从机是否应答（NACK = 从机不存在） */
    if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF))
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_AF);
        return 1;
    }

    /* 逐字节接收数据 */
    for (uint32_t i = 0; i < len; i++)
    {
        /* 最后一个字节发送 NACK，通知从机不再发送 */
        if (i == len - 1)
            I2C_AcknowledgeConfig(I2C2, DISABLE);

        /* 等待数据接收完成 */
        i2c2_wait_flag(I2C_EVENT_MASTER_BYTE_RECEIVED, timeout_ms);
        buf[i] = I2C_ReceiveData(I2C2);
    }

    /* 发送停止信号 */
    I2C_GenerateSTOP(I2C2, ENABLE);
    return 0;
}

/**
 * @brief I2C 主机写数据后重复 START 读数据（组合事务）
 *
 * @param addr7   7 位从机地址
 * @param wdata   待写入的数据
 * @param wlen    写入字节数
 * @param rbuf    接收缓冲区
 * @param rlen    期望接收的字节数
 * @return        0=成功，1=超时/NACK/总线错误
 *
 * 时序：START → Addr+W → 数据 → Sr(重复START) → Addr+R → 数据 → NACK → STOP
 * 用于 BME280 等需要写寄存器地址后重复 START 读数据的传感器。
 */
uint8_t i2c2_write_read(uint8_t addr7, const uint8_t *wdata, uint32_t wlen, uint8_t *rbuf, uint32_t rlen)
{
    uint32_t timeout_ms = 50;

    /* 等待总线空闲 */
    uint32_t t0 = time_ms64();
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        if ((time_ms64() - t0) > timeout_ms) { i2c2_bus_reset(); return 1; }
    }

    /* 发送起始信号 */
    I2C_GenerateSTART(I2C2, ENABLE);
    i2c2_wait_flag(I2C_EVENT_MASTER_MODE_SELECT, timeout_ms);

    /* 发送从机地址 + 写位 */
    I2C_Send7bitAddress(I2C2, addr7 << 1, I2C_Direction_Transmitter);
    i2c2_wait_flag(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout_ms);

    /* 检查 NACK */
    if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF))
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_AF);
        return 1;
    }

    /* 逐字节发送写数据 */
    for (uint32_t i = 0; i < wlen; i++)
    {
        I2C_SendData(I2C2, wdata[i]);
        i2c2_wait_flag(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout_ms);
    }

    /* 重复 START（Sr） */
    I2C_GenerateSTART(I2C2, ENABLE);
    i2c2_wait_flag(I2C_EVENT_MASTER_MODE_SELECT, timeout_ms);

    /* 发送从机地址 + 读位 */
    I2C_Send7bitAddress(I2C2, addr7 << 1, I2C_Direction_Receiver);
    i2c2_wait_flag(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, timeout_ms);

    /* 检查 NACK */
    if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF))
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_AF);
        return 1;
    }

    /* 使能应答 */
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    /* 逐字节接收数据 */
    for (uint32_t i = 0; i < rlen; i++)
    {
        /* 最后一个字节发送 NACK */
        if (i == rlen - 1)
            I2C_AcknowledgeConfig(I2C2, DISABLE);

        i2c2_wait_flag(I2C_EVENT_MASTER_BYTE_RECEIVED, timeout_ms);
        rbuf[i] = I2C_ReceiveData(I2C2);
    }

    /* 发送停止信号 */
    I2C_GenerateSTOP(I2C2, ENABLE);
    return 0;
}
