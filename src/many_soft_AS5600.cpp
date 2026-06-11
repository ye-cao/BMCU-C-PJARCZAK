#include "many_soft_AS5600.h"

#include "hal/time_hw.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#include "core_riscv.h"

/** @brief I2C位延迟的tick数，由init()根据系统时钟计算(默认4us) */
static uint32_t g_iic_delay_ticks = 1;

/** @brief I2C时序延迟宏，调用delayTicks32实现指定tick数的忙等待 */
#define iic_delay() do { delayTicks32(g_iic_delay_ticks); } while (0)

// --- AS5600 I2C地址定义 ---
/** @brief AS5600写地址 = 0x36左移1位 = 0x6C(7位地址0x36，最低位R/W=0) */
#define AS5600_write_address (0x36 << 1)

/** @brief AS5600读地址 = 0x36左移1位+1 = 0x6D(7位地址0x36，最低位R/W=1) */
#define AS5600_read_address  ((0x36 << 1) + 1)

// --- AS5600寄存器地址 ---
/** @brief 原始角度寄存器地址(12位角度值，只读) */
#define AS5600_raw_angle 0x0C

/** @brief 状态寄存器地址(包含磁铁检测标志位) */
#define AS5600_status    0x0B

/**
 * @brief GPIO引脚置高(设置BSHR寄存器)
 * @param p  GPIO端口基地址
 * @param pin 引脚掩码(如GPIO_Pin_0)
 */
static inline void gpio_hi(GPIO_TypeDef* p, uint16_t pin) { p->BSHR = pin; }

/**
 * @brief GPIO引脚置低(设置BCR寄存器)
 * @param p  GPIO端口基地址
 * @param pin 引脚掩码(如GPIO_Pin_0)
 */
static inline void gpio_lo(GPIO_TypeDef* p, uint16_t pin) { p->BCR  = pin; }

/**
 * @brief 计算16位引脚掩码中最低有效位的位置(0~15)
 * @param pinMask 引脚掩码(仅1位有效，如0x0001、0x0002等)
 * @return 最低有效位的位索引
 * @details 使用编译器内建函数__builtin_ctz(Count Trailing Zeros)计算尾部零的个数
 */
static inline uint32_t pin_index_u16(uint16_t pinMask)
{
    return (uint32_t)__builtin_ctz((uint32_t)pinMask);
}

/**
 * @brief 配置GPIO引脚的4位配置字段(CFGLR/CFGHR)
 * @param p       GPIO端口基地址
 * @param pinMask 引脚掩码
 * @param cfg4    4位配置值(如0x7=开漏输出50MHz，0x8=上拉输入)
 * @details CH32V203的GPIO配置寄存器每个引脚占4位(CNF[1:0]+MODE[1:0])，
 *          低8引脚在CFGLR(偏移0x00)，高8引脚在CFGHR(偏移0x04)
 */
static inline void gpio_set_cfg4(GPIO_TypeDef* p, uint16_t pinMask, uint32_t cfg4)
{
    uint32_t pin = pin_index_u16(pinMask);
    volatile uint32_t* cfg = (pin < 8) ? &p->CFGLR : &p->CFGHR;
    uint32_t shift = (pin & 7u) * 4u;
    uint32_t m = 0xFu << shift;
    uint32_t v = *cfg;
    v = (v & ~m) | ((cfg4 & 0xFu) << shift);
    *cfg = v;
}

/**
 * @brief 构造函数 - 初始化所有缓冲区和指针
 * @details 将通道计数清零，所有公共/私有缓冲区初始化为零值/离线状态，
 *          所有指针指向内部静态缓冲区
 */
AS5600_soft_IIC_many::AS5600_soft_IIC_many()
{
    numbers   = 0;

    // 公共数据指针指向内部缓冲区
    online     = online_buf;
    magnet_stu = magnet_buf;
    raw_angle  = raw_buf;
    data       = data_buf;

    // 私有数据指针指向内部缓冲区
    error    = error_buf;
    port_SDA = port_SDA_buf;
    port_SCL = port_SCL_buf;
    pin_SDA  = pin_SDA_buf;
    pin_SCL  = pin_SCL_buf;

    for (int i = 0; i < kMax; i++)
    {
        online_buf[i] = false;      /* 默认离线 */
        magnet_buf[i] = offline;    /* 磁铁未检测 */
        raw_buf[i]    = 0;          /* 角度清零 */
        data_buf[i]   = 0;          /* 数据清零 */
        error_buf[i]  = 0;          /* 无错误 */

        port_SDA_buf[i] = nullptr;  /* SDA端口未配置 */
        port_SCL_buf[i] = nullptr;  /* SCL端口未配置 */
        pin_SDA_buf[i]  = 0;        /* SDA引脚掩码清零 */
        pin_SCL_buf[i]  = 0;        /* SCL引脚掩码清零 */
    }
}

/**
 * @brief 析构函数 - 当前无动态内存，为空
 */
AS5600_soft_IIC_many::~AS5600_soft_IIC_many()
{
    // 所有缓冲区为静态数组，无需释放
}

/**
 * @brief 使能指定GPIO端口的外设时钟
 * @param p GPIO端口指针(GPIOA/B/C/D)
 * @details CH32V203 GPIO挂载在APB2总线上，操作前必须使能对应时钟
 */
void AS5600_soft_IIC_many::enable_gpio_clock(GPIO_TypeDef* p)
{
    if      (p == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (p == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (p == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (p == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

/**
 * @brief 将第i通道的SDA引脚切换为上拉输入模式
 * @param i 通道索引(0~kMax-1)
 * @details 用于I2C读取阶段: 先ODR置高(使能内部上拉)，再配置CNF/MODE=0b1000(浮空/上拉输入)
 */
void AS5600_soft_IIC_many::sda_mode_ipu(int i)
{
    gpio_hi(port_SDA[i], pin_SDA[i]);
    gpio_set_cfg4(port_SDA[i], pin_SDA[i], 0x8u); // CNF=10, MODE=00 = 上拉输入
}

/**
 * @brief 将第i通道的SDA引脚切换为开漏输出模式
 * @param i 通道索引(0~kMax-1)
 * @details 用于I2C发送阶段: CNF=01, MODE=11 → 0b0111(开漏输出50MHz)
 *          切换后立即释放SDA为高电平
 */
void AS5600_soft_IIC_many::sda_mode_od(int i)
{
    gpio_set_cfg4(port_SDA[i], pin_SDA[i], 0x7u); // CNF=01, MODE=11 = 开漏输出50MHz
    gpio_hi(port_SDA[i], pin_SDA[i]); // 释放总线为高
}

/**
 * @brief 将所有有效通道的指定端口/引脚数组置高
 * @param port 端口指针数组
 * @param pin  引脚掩码数组
 * @details 仅操作error[i]==0(未出错)的通道，通过BSHR寄存器原子置位
 */
void AS5600_soft_IIC_many::set_h(GPIO_TypeDef* const* port, const uint16_t* pin)
{
    for (int i = 0; i < numbers; i++)
        if (error[i] == 0) port[i]->BSHR = pin[i];
}

/**
 * @brief 将所有有效通道的指定端口/引脚数组置低
 * @param port 端口指针数组
 * @param pin  引脚掩码数组
 * @details 仅操作error[i]==0(未出错)的通道，通过BCR寄存器原子清零
 */
void AS5600_soft_IIC_many::set_l(GPIO_TypeDef* const* port, const uint16_t* pin)
{
    for (int i = 0; i < numbers; i++)
        if (error[i] == 0) port[i]->BCR = pin[i];
}

/**
 * @brief 初始化多路AS5600软I2C驱动
 * @param GPIO_SCL_port  各通道SCL引脚的GPIO端口指针数组
 * @param GPIO_SCL_pin   各通道SCL引脚的引脚掩码数组
 * @param GPIO_SDA_port  各通道SDA引脚的GPIO端口指针数组
 * @param GPIO_SDA_pin   各通道SDA引脚的引脚掩码数组
 * @param num            需要初始化的通道数量(超出kMax则截断为kMax)
 * @details 初始化流程:
 *          1. 校验通道数，计算I2C延迟tick数(4us × 每us的tick数)
 *          2. 保存各通道的GPIO配置信息
 *          3. 使能各通道GPIO端口时钟
 *          4. 调用init_iic()配置引脚模式
 *          5. 调用updata_stu()检测磁铁状态
 */
void AS5600_soft_IIC_many::init(GPIO_TypeDef* const* GPIO_SCL_port, const uint16_t* GPIO_SCL_pin,
                                GPIO_TypeDef* const* GPIO_SDA_port, const uint16_t* GPIO_SDA_pin,
                                int num)
{
    if (num <= 0) { numbers = 0; return; }
    if (num > kMax) num = kMax;
    numbers = num;

    // 根据系统时钟计算4us对应的tick数，至少为1
    g_iic_delay_ticks = 4u * time_hw_ticks_per_us();
    if (!g_iic_delay_ticks) g_iic_delay_ticks = 1;

    for (int i = 0; i < numbers; i++)
    {
        port_SCL[i] = GPIO_SCL_port[i];
        pin_SCL[i]  = GPIO_SCL_pin[i];

        port_SDA[i] = GPIO_SDA_port[i];
        pin_SDA[i]  = GPIO_SDA_pin[i];

        magnet_stu[i] = offline;
        online[i]     = false;
        raw_angle[i]  = 0;
        data[i]       = 0;
        error[i]      = 0;

        // 使能SCL和SDA所在GPIO端口的时钟
        enable_gpio_clock(port_SCL[i]);
        enable_gpio_clock(port_SDA[i]);
    }

    init_iic();     /* 配置GPIO引脚模式并初始化I2C空闲状态 */
    updata_stu();   /* 首次检测磁铁状态 */
}

/**
 * @brief 清除所有通道的错误标志和数据缓冲区
 * @details 将error[i]和data[i]均置0，为下一次I2C操作做准备
 */
void AS5600_soft_IIC_many::clear_datas()
{
    for (int i = 0; i < numbers; i++)
    {
        error[i] = 0;
        data[i]  = 0;
    }
}

/**
 * @brief 初始化I2C引脚的GPIO模式和空闲状态
 * @details 对每个通道:
 *          1. SCL/SDA先置高(空闲态)
 *          2. SCL配置为推挽输出50MHz(GPIO_Mode_Out_PP)
 *          3. SDA配置为开漏输出50MHz(GPIO_Mode_Out_OD)
 *          4. 再次确认SCL/SDA为高(空闲态)
 *          5. 清除错误标志
 */
void AS5600_soft_IIC_many::init_iic()
{
    for (int i = 0; i < numbers; i++)
    {
        gpio_hi(port_SCL[i], pin_SCL[i]);
        gpio_hi(port_SDA[i], pin_SDA[i]);

        GPIO_InitTypeDef gi = {0};
        gi.GPIO_Speed = GPIO_Speed_50MHz;

        // SCL: 推挽输出，主机主动驱动时钟
        gi.GPIO_Mode = GPIO_Mode_Out_PP;
        gi.GPIO_Pin  = pin_SCL[i];
        GPIO_Init(port_SCL[i], &gi);

        // SDA: 开漏输出，允许从机拉低(线与逻辑)
        gi.GPIO_Mode = GPIO_Mode_Out_OD;
        gi.GPIO_Pin  = pin_SDA[i];
        GPIO_Init(port_SDA[i], &gi);

        // 空闲态: SCL和SDA均为高
        gpio_hi(port_SCL[i], pin_SCL[i]);
        gpio_hi(port_SDA[i], pin_SDA[i]);

        error[i] = 0;
    }
}

/**
 * @brief 发送I2C起始条件并发送从机地址字节
 * @param ADR 从机地址(左移1位后的值，最低位含R/W标志)
 * @details I2C START时序:
 *          1. SDA和SCL均为高(空闲)
 *          2. SCL保持高时，SDA由高→低 = START信号
 *          3. SCL拉低，准备发送数据
 *          4. 调用write_iic()发送地址字节
 */
void AS5600_soft_IIC_many::start_iic(unsigned char ADR)
{
    iic_delay();
    set_h(port_SDA, pin_SDA);   // SDA=H
    set_h(port_SCL, pin_SCL);   // SCL=H
    iic_delay();
    set_l(port_SDA, pin_SDA);   // SDA=L (START: SCL为高时SDA下降沿)
    iic_delay();
    set_l(port_SCL, pin_SCL);   // SCL=L，准备发送地址

    write_iic((uint8_t)ADR);    // 发送从机地址字节
}

/**
 * @brief 发送I2C停止条件，强制恢复总线到空闲态
 * @details I2C STOP时序:
 *          1. SCL和SDA先拉低(确保起始条件)
 *          2. SCL由低→高
 *          3. SDA由低→高 = STOP信号(SCL为高时SDA上升沿)
 *          注意: 无论error状态如何，都会对所有通道执行STOP操作，确保总线不被锁死
 */
void AS5600_soft_IIC_many::stop_iic()
{
    // 先将SCL和SDA都拉低(准备STOP条件)
    for (int i = 0; i < numbers; i++) {
        gpio_lo(port_SCL[i], pin_SCL[i]);
        gpio_lo(port_SDA[i], pin_SDA[i]);
    }
    iic_delay();

    // SCL拉高
    for (int i = 0; i < numbers; i++) {
        gpio_hi(port_SCL[i], pin_SCL[i]);
    }
    iic_delay();

    // SDA拉高 = STOP条件(SCL高电平期间SDA上升沿)
    for (int i = 0; i < numbers; i++) {
        gpio_hi(port_SDA[i], pin_SDA[i]);
    }
    iic_delay();
}

/**
 * @brief 通过I2C发送一个字节(MSB先发)
 * @param byte 待发送的字节数据
 * @details 从最高位(MSB)开始逐位发送:
 *          1. 设置SDA电平(0或1)
 *          2. SCL拉高 → 从机采样
 *          3. SCL拉低 → 准备下一位
 *          发送8位后自动调用wait_ack_iic()等待从机应答
 */
void AS5600_soft_IIC_many::write_iic(uint8_t byte)
{
    for (uint8_t m = 0x80; m; m >>= 1)  // 从bit7(MSB)到bit0
    {
        iic_delay();
        if (byte & m) set_h(port_SDA, pin_SDA);  // 数据位=1
        else          set_l(port_SDA, pin_SDA);  // 数据位=0

        set_h(port_SCL, pin_SCL);   // SCL高电平 → 从机采样数据
        iic_delay();
        set_l(port_SCL, pin_SCL);   // SCL低电平 → 准备下一位
    }
    wait_ack_iic();  // 8位发送完毕，等待ACK
}

/**
 * @brief 通过I2C接收一个字节(MSB先收)
 * @param ack true=发送ACK(主机应答，表示还要继续读)，false=发送NACK(最后一个字节)
 * @details 接收流程:
 *          1. 将所有有效通道的SDA切换为上拉输入模式
 *          2. 逐位读取: SCL高电平时采样SDA，读入data[]缓冲区
 *          3. 8位读完后，将SDA切回开漏输出模式
 *          4. 发送ACK(NACK): SDA=LOW(ACK)或SDA=HIGH(NACK)，产生SCL时钟
 *          数据通过位移操作存入data[j]缓冲区
 */
void AS5600_soft_IIC_many::read_iic(bool ack)
{
    // SDA切换为上拉输入模式(准备接收数据)
    for (int i = 0; i < numbers; i++)
        if (error[i] == 0) sda_mode_ipu(i);

    for (int bit = 0; bit < 8; bit++)
    {
        iic_delay();
        set_h(port_SCL, pin_SCL);   // SCL高 → 从机驱动SDA
        iic_delay();

        for (int j = 0; j < numbers; j++)
        {
            data[j] <<= 1;  // 左移一位，准备接收新位
            if (port_SDA[j]->INDR & pin_SDA[j]) data[j] |= 0x01;  // 读取SDA引脚电平
        }

        set_l(port_SCL, pin_SCL);   // SCL低 → 准备下一位
    }

    // SDA切回开漏输出模式(准备发送ACK/NACK)
    for (int i = 0; i < numbers; i++)
        sda_mode_od(i);

    iic_delay();
    // 发送ACK或NACK
    if (ack) set_l(port_SDA, pin_SDA);  // ACK: SDA=LOW
    else     set_h(port_SDA, pin_SDA);  // NACK: SDA=HIGH

    iic_delay();
    set_h(port_SCL, pin_SCL);   // 产生ACK/NACK时钟脉冲
    iic_delay();
    set_l(port_SCL, pin_SCL);
    iic_delay();
}

/**
 * @brief 等待从机应答(ACK/NACK检测)
 * @details 流程:
 *          1. SDA释放为高(主机不再驱动)
 *          2. 将SDA切换为上拉输入模式
 *          3. SCL拉高，采样SDA电平
 *          4. SDA=LOW → ACK(从机正常应答)
 *          5. SDA=HIGH → NACK(从机无应答)，设置error[i]=1
 *          6. SCL拉低，SDA切回开漏输出
 */
void AS5600_soft_IIC_many::wait_ack_iic()
{
    set_h(port_SDA, pin_SDA);   // 释放SDA

    // SDA切换为输入(在SCL拉高之前)
    for (int i = 0; i < numbers; i++)
        if (error[i] == 0) sda_mode_ipu(i);

    iic_delay();
    set_h(port_SCL, pin_SCL);   // SCL高 → 从机驱动SDA作为应答
    iic_delay();

    // 检测各通道的ACK/NACK
    for (int i = 0; i < numbers; i++) {
        if (error[i] == 0) {
            if (port_SDA[i]->INDR & pin_SDA[i]) error[i] = 1; // NACK → 标记错误
        }
    }

    set_l(port_SCL, pin_SCL);   // SCL低
    iic_delay();

    // SDA切回开漏输出模式
    for (int i = 0; i < numbers; i++)
        sda_mode_od(i);
}

/**
 * @brief 读取AS5600的8位寄存器(如状态寄存器0x0B)
 * @param reg 目标寄存器地址
 * @details I2C通信序列:
 *          1. clear_datas() - 清除缓冲区
 *          2. [START][从机写地址0x6C] → 写模式
 *          3. [寄存器地址] → 指定要读的寄存器
 *          4. [RESTART][从机读地址0x6D] → 切换到读模式
 *          5. [读1字节][NACK] → 读取数据并发送NACK(只读1字节)
 *          6. [STOP] → 结束通信
 */
void AS5600_soft_IIC_many::read_reg8(uint8_t reg)
{
    if (!numbers) return;

    clear_datas();
    start_iic(AS5600_write_address);    // [START][写地址]
    write_iic(reg);                      // [寄存器地址]
    start_iic(AS5600_read_address);     // [RESTART][读地址]
    read_iic(false);                     // 读1字节 + NACK
    stop_iic();                          // [STOP]
}

/**
 * @brief 读取AS5600的16位寄存器(如原始角度寄存器0x0C)
 * @param reg 目标寄存器地址
 * @details I2C通信序列:
 *          1. clear_datas() - 清除缓冲区
 *          2. [START][从机写地址0x6C] → 写模式
 *          3. [寄存器地址] → 指定要读的寄存器
 *          4. [RESTART][从机读地址0x6D] → 切换到读模式
 *          5. [读高字节][ACK] → 继续读
 *          6. [读低字节][NACK] → 最后一字节，发送NACK
 *          7. [STOP] → 结束通信
 *          AS5600的16位数据中，高4位通常为0，低12位为角度值
 */
void AS5600_soft_IIC_many::read_reg16(uint8_t reg)
{
    if (!numbers) return;

    clear_datas();
    start_iic(AS5600_write_address);    // [START][写地址]
    write_iic(reg);                      // [寄存器地址]
    start_iic(AS5600_read_address);     // [RESTART][读地址]
    read_iic(true);                      // 读高字节 + ACK(继续读)
    read_iic(false);                     // 读低字节 + NACK(最后一字节)
    stop_iic();                          // [STOP]
}

/**
 * @brief 更新所有通道的磁铁状态和在线标志
 * @details 读取AS5600状态寄存器(0x0B)后解析各状态位:
 *          - data[i] & 0x20 → 磁铁检测标志(1=检测到磁铁，0=未检测到)
 *          - data[i] & 0x10 → 磁铁过近(磁场太强，AGC增益过低)
 *          - data[i] & 0x08 → 磁铁过远(磁场太弱，AGC增益过高)
 *          - 两者都不为0 → 磁铁位置正常
 *          同时更新online[i]标志(error[i]==0 → 在线)
 */
void AS5600_soft_IIC_many::updata_stu()
{
    read_reg8(AS5600_status);   // 读取状态寄存器(0x0B)

    for (int i = 0; i < numbers; i++)
    {
        online[i] = (error[i] == 0);   // I2C通信无错误 = 传感器在线

        if (!(data[i] & 0x20)) magnet_stu[i] = offline;    // bit5=0 → 磁铁未检测到
        else
        {
            if      (data[i] & 0x10) magnet_stu[i] = low;   // bit4=1 → 磁铁过近
            else if (data[i] & 0x08) magnet_stu[i] = high;  // bit3=1 → 磁铁过远
            else                     magnet_stu[i] = normal; // 正常范围
        }
    }
}

/**
 * @brief 更新所有通道的原始角度值和在线标志
 * @details 读取AS5600原始角度寄存器(0x0C)获取12位角度值(0~4095)
 *          - 读取成功(error[i]==0): raw_angle[i]=data[i], online[i]=true
 *          - 读取失败(error[i]!=0): raw_angle[i]=0, online[i]=false
 *          原始角度值代表磁铁的绝对旋转角度，每圈4096个计数
 */
void AS5600_soft_IIC_many::updata_angle()
{
    read_reg16(AS5600_raw_angle);   // 读取16位原始角度寄存器(0x0C)

    for (int i = 0; i < numbers; i++)
    {
        if (error[i] == 0)
        {
            raw_angle[i] = data[i];     // 保存角度值(12位有效)
            online[i] = true;
        }
        else
        {
            raw_angle[i] = 0;           // 读取失败，角度清零
            online[i] = false;
        }
    }
}
