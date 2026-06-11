#pragma once
#include <stdint.h>

#include "ch32v20x_gpio.h"
#include "ch32v20x.h"

/**
 * @brief AS5600软I2C多通道驱动类
 * @details 通过GPIO位操作(bit-bang)方式同时驱动最多4路AS5600磁旋转编码器。
 *          每路使用独立的SCL/SDA引脚对，通过寄存器直接操作实现I2C时序。
 *          SCL引脚: PB15/PB14/PB13/PB12
 *          SDA引脚: PD0/PC15/PC14/PC13
 *          AS5600 I2C地址: 0x36，寄存器0x0B(状态)和0x0C(原始角度)
 */
class AS5600_soft_IIC_many
{
public:
    /**
     * @brief AS5600磁铁状态枚举
     * @details 表示磁铁相对传感器的检测状态
     */
    enum _AS5600_magnet_stu
    {
        low     = 1,    /**< 磁铁过近(磁场强度过高)，AGC增益过低 */
        high    = 2,    /**< 磁铁过远(磁场强度过低)，AGC增益过高 */
        offline = -1,   /**< 磁铁未检测到(离线状态) */
        normal  = 0     /**< 磁铁位置正常，磁场强度在有效范围内 */
    };

    /** @brief 构造函数，初始化所有内部缓冲区为默认值 */
    AS5600_soft_IIC_many();

    /** @brief 析构函数(当前无动态内存释放操作) */
    ~AS5600_soft_IIC_many();

    /**
     * @brief 初始化多路AS5600软I2C
     * @param GPIO_SCL_port  SCL引脚所在的GPIO端口指针数组
     * @param GPIO_SCL_pin   SCL引脚的引脚掩码数组
     * @param GPIO_SDA_port  SDA引脚所在的GPIO端口指针数组
     * @param GPIO_SDA_pin   SDA引脚的引脚掩码数组
     * @param num            需要初始化的通道数量(1~4)
     * @details 配置GPIO时钟、引脚模式，并初始化I2C总线，最后自动检测磁铁状态
     */
    void init(GPIO_TypeDef* const* GPIO_SCL_port, const uint16_t* GPIO_SCL_pin,
              GPIO_TypeDef* const* GPIO_SDA_port, const uint16_t* GPIO_SDA_pin,
              int num);

    /**
     * @brief 更新所有通道的磁铁状态
     * @details 读取AS5600状态寄存器(0x0B)，判断磁铁是否在线及磁场强度是否正常
     *          并更新online[]和magnet_stu[]数组
     */
    void updata_stu();

    /**
     * @brief 更新所有通道的原始角度值
     * @details 读取AS5600原始角度寄存器(0x0C)，将12位角度值保存到raw_angle[]
     *          同时更新online[]状态，读取失败的通道角度置0
     */
    void updata_angle();

    /** @brief 各通道在线状态数组指针(指向online_buf)，true=传感器响应I2C */
    bool*               online;

    /** @brief 各通道磁铁状态数组指针(指向magnet_buf) */
    _AS5600_magnet_stu* magnet_stu;

    /** @brief 各通道原始角度值数组指针(指向raw_buf)，12位(0~4095) */
    uint16_t*           raw_angle;

    /** @brief I2C读取的原始数据缓冲区数组指针(指向data_buf) */
    uint16_t*           data;

    /** @brief 当前初始化的通道数量 */
    int                 numbers;

private:
    /** @brief 支持的最大通道数(硬编码为4) */
    static constexpr int kMax = 4;

    // --- 内部缓冲区 ---
    /** @brief 在线状态缓冲区，每通道一个bool */
    bool               online_buf[kMax];

    /** @brief 磁铁状态缓冲区，每通道一个枚举值 */
    _AS5600_magnet_stu magnet_buf[kMax];

    /** @brief 原始角度缓冲区，每通道一个uint16_t */
    uint16_t           raw_buf[kMax];

    /** @brief I2C读取原始数据缓冲区，每通道一个uint16_t */
    uint16_t           data_buf[kMax];

    /** @brief 错误标志缓冲区，0=正常，1=NACK(无应答) */
    int                error_buf[kMax];

    // --- GPIO配置缓冲区 ---
    /** @brief SDA引脚所在GPIO端口指针数组 */
    GPIO_TypeDef*      port_SDA_buf[kMax];

    /** @brief SCL引脚所在GPIO端口指针数组 */
    GPIO_TypeDef*      port_SCL_buf[kMax];

    /** @brief SDA引脚掩码数组(如GPIO_Pin_0) */
    uint16_t           pin_SDA_buf[kMax];

    /** @brief SCL引脚掩码数组(如GPIO_Pin_12) */
    uint16_t           pin_SCL_buf[kMax];

    // --- 内部指针(指向对应缓冲区) ---
    /** @brief 错误标志数组指针 */
    int*          error;

    /** @brief SDA端口指针数组 */
    GPIO_TypeDef** port_SDA;

    /** @brief SCL端口指针数组 */
    GPIO_TypeDef** port_SCL;

    /** @brief SDA引脚掩码数组指针 */
    uint16_t*      pin_SDA;

    /** @brief SCL引脚掩码数组指针 */
    uint16_t*      pin_SCL;

    /**
     * @brief 初始化所有通道的I2C GPIO引脚
     * @details SCL配置为推挽输出50MHz，SDA配置为开漏输出50MHz
     *          初始化后SCL/SDA均置高(空闲状态)
     */
    void init_iic();

    /**
     * @brief 发送I2C起始条件并发送从机地址
     * @param ADR 7位从机地址左移1位后的值(含读/写位)
     * @details 时序: SDA由高→低(SCL为高时)=START，然后发送地址字节
     */
    void start_iic(unsigned char ADR);

    /**
     * @brief 发送I2C停止条件
     * @details 时序: SCL由低→高，SDA由低→高=STOP
     *          无论error状态如何都会强制将总线恢复到空闲状态
     */
    void stop_iic();

    /**
     * @brief 通过I2C发送一个字节(8位，MSB先发)
     * @param byte 待发送的字节数据
     * @details 每发一位后产生时钟脉冲，发送完毕后自动等待ACK
     */
    void write_iic(uint8_t byte);

    /**
     * @brief 通过I2C接收一个字节(8位，MSB先收)
     * @param ack true=发送ACK(应答)，false=发送NACK(非应答，最后一个字节)
     * @details 接收前将SDA切换为上拉输入模式，接收完毕后切回开漏输出发送ACK/NACK
     *          数据保存在data[]缓冲区中
     */
    void read_iic(bool ack);

    /**
     * @brief 等待从机应答(ACK/NACK)
     * @details SDA释放(高)，产生SCL时钟，采样SDA电平:
     *          SDA=0 → ACK(从机正常应答)
     *          SDA=1 → NACK(从机无应答，设置error[i]=1)
     */
    void wait_ack_iic();

    /**
     * @brief 清除所有通道的错误标志和数据缓冲区
     * @details 将error[i]和data[i]均置0
     */
    void clear_datas();

    /**
     * @brief 读取AS5600的8位寄存器
     * @param reg 寄存器地址(如0x0B状态寄存器)
     * @details I2C流程: [START][写地址+写][寄存器地址][RESTART][写地址+读][读1字节][NACK][STOP]
     *          读取结果保存在data[]中
     */
    void read_reg8(uint8_t reg);

    /**
     * @brief 读取AS5600的16位寄存器(2字节)
     * @param reg 寄存器地址(如0x0C原始角度寄存器)
     * @details I2C流程: [START][写地址+写][寄存器地址][RESTART][写地址+读][读高字节+ACK][读低字节+NACK][STOP]
     *          AS5600返回的数据为16位，其中12位有效(角度值)
     */
    void read_reg16(uint8_t reg);

    /**
     * @brief 使能指定GPIO端口的时钟
     * @param p GPIO端口指针(GPIOA/B/C/D)
     * @details CH32V203需要先使能GPIO外设时钟才能操作引脚
     */
    void enable_gpio_clock(GPIO_TypeDef* p);

    /**
     * @brief 将SDA引脚切换为上拉输入模式(用于读取从机数据)
     * @param i 通道索引(0~3)
     * @details ODR置1，CNF/MODE配置为0b1000(上拉输入)
     */
    void sda_mode_ipu(int i);

    /**
     * @brief 将SDA引脚切换为开漏输出模式(用于发送数据)
     * @param i 通道索引(0~3)
     * @details CNF=01, MODE=11 → 0b0111(开漏输出50MHz)，随后释放SDA为高
     */
    void sda_mode_od(int i);

    /**
     * @brief 将所有有效通道的指定引脚置高
     * @param port GPIO端口指针数组
     * @param pin  引脚掩码数组
     * @details 仅对error[i]==0(正常)的通道操作，通过BSHR寄位置位
     */
    void set_h(GPIO_TypeDef* const* port, const uint16_t* pin);

    /**
     * @brief 将所有有效通道的指定引脚置低
     * @param port GPIO端口指针数组
     * @param pin  引脚掩码数组
     * @details 仅对error[i]==0(正常)的通道操作，通过BCR寄存器清零
     */
    void set_l(GPIO_TypeDef* const* port, const uint16_t* pin);
};
