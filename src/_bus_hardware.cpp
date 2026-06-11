#include "_bus_hardware.h"

#include "ch32v20x.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_usart.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_misc.h"
#include "core_riscv.h"
#include "crc_bus.h"

/**
 * @brief 当前连接的主机设备类型
 * 
 * 记录当前与本 AMS 通信的主机设备类型：
 * - 0x0000: 无设备连接
 * - 0x0001: AHUB 集线器
 * - 0x0700: AMS（打印机直连）
 * 
 * 用于长帧通信中的地址过滤和协议分发。
 */
uint16_t bus_host_device_type=0x0000;

/** @brief USART1 DMA 发送初始化结构体（全局复用） */
DMA_InitTypeDef bus_uart1_dma_init_structure;

/** @brief USART1 外设初始化函数声明 */
void bus_uart1_init();

/**
 * @brief USART1 DMA 发送函数声明
 * 
 * 通过 DMA1 Channel4 将数据从内存发送到 USART1 数据寄存器。
 * 支持半双工通信，通过 DE（Driver Enable）引脚控制收发方向。
 * 
 * @param data 待发送的数据指针（需 4 字节对齐）
 * @param length 数据长度（字节）
 */
void bus_uart1_dma_send(uint8_t *data, uint16_t length);


/**
 * @brief 全局总线端口处理实例
 * 
 * 管理与主机（打印机或集线器）的全部串口通信。
 * 包括数据包接收解析、CRC 校验、心跳检测和响应发送。
 */
_bus_port_deal bus_port_to_host;

/** @brief 将 UART1 接收中断映射到总线端口的 IRQ 处理函数 */
#define uart1_port_irq(data) bus_port_to_host.irq(data)
/** @brief 将 UART1 空闲标志映射到总线端口的 idle 标志 */
#define uart1_port_idle bus_port_to_host.idle
/** @brief 将总线端口的发送函数映射到 UART1 DMA 发送函数 */
#define bus_port_to_host_send_func bus_uart1_dma_send

/**
 * @brief 初始化总线硬件和协议栈
 * 
 * 执行以下初始化步骤：
 * 1. 启用 CRC 硬件外设时钟
 * 2. 初始化 CRC 算法（CRC8 和 CRC16 查找表）
 * 3. 初始化总线端口处理模块（双缓冲、状态机等）
 * 4. 初始化 USART1 外设（GPIO、UART、DMA、中断）
 */
void bus_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
    bus_crc_init();
    bus_port_to_host.init(bus_port_to_host_send_func);
    bus_uart1_init();
}

/**
 * @brief 初始化 USART1 外设
 * 
 * 配置 CH32V203 的 USART1 用于 1.25Mbps 半双工串口通信：
 * 
 * GPIO 配置：
 * - PA9:  USART1 TX（复用推挽输出，50MHz）
 * - PA10: USART1 RX（上拉输入）
 * - PA12: DE（驱动器使能，推挽输出，50MHz）
 *        DE=高电平：发送模式；DE=低电平：接收模式
 * 
 * USART 配置：
 * - 波特率：1,250,000 bps
 * - 数据位：9 位（含偶校验位）
 * - 停止位：1 位
 * - 校验：偶校验
 * - 模式：收发双工
 * 
 * 中断配置：
 * - RXNE（接收缓冲区非空）：逐字节接收数据
 * - TC（传输完成）：DMA 发送完成后切换回接收模式
 * - 优先级：最高（抢占优先级 0，子优先级 0）
 * 
 * DMA 配置：
 * - DMA1 Channel4：USART1 TX
 * - 方向：内存 → 外设
 * - 模式：单次传输（Normal）
 * - 优先级：非常高
 * - 数据大小：字节
 */
void bus_uart1_init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    // 启用外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* USART1 TX-->A.9   RX-->A.10   DE-->A.12*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX 引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX 引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    // 上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // DE 引脚（驱动器使能）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BCR = GPIO_Pin_12; // 初始状态：DE=低电平（接收模式）

    // USART1 参数配置
    USART_InitStructure.USART_BaudRate = 1250000;              // 波特率 1.25Mbps
    USART_InitStructure.USART_WordLength = USART_WordLength_9b; // 9 位数据（含校验位）
    USART_InitStructure.USART_StopBits = USART_StopBits_1;     // 1 位停止位
    USART_InitStructure.USART_Parity = USART_Parity_Even;      // 偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // 收发模式

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 启用接收中断
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);   // 启用传输完成中断

    // NVIC 中断配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 最高抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 最高子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // DMA1 Channel4 配置（USART1 TX）
    bus_uart1_dma_init_structure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR; // 外设地址
    bus_uart1_dma_init_structure.DMA_MemoryBaseAddr = (uint32_t)0;                    // 内存地址（运行时设置）
    bus_uart1_dma_init_structure.DMA_DIR = DMA_DIR_PeripheralDST;                     // 方向：内存→外设
    bus_uart1_dma_init_structure.DMA_Mode = DMA_Mode_Normal;                          // 单次传输模式
    bus_uart1_dma_init_structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;       // 外设地址不递增
    bus_uart1_dma_init_structure.DMA_MemoryInc = DMA_MemoryInc_Enable;                // 内存地址递增
    bus_uart1_dma_init_structure.DMA_Priority = DMA_Priority_VeryHigh;                // 非常高优先级
    bus_uart1_dma_init_structure.DMA_M2M = DMA_M2M_Disable;                           // 非内存到内存
    bus_uart1_dma_init_structure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        // 内存数据大小：字节
    bus_uart1_dma_init_structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据大小：字节
    bus_uart1_dma_init_structure.DMA_BufferSize = 0;                                  // 缓冲区大小（运行时设置）
    DMA_Init(DMA1_Channel4, &bus_uart1_dma_init_structure);

    USART_Cmd(USART1, ENABLE); // 启用 USART1
}

/**
 * @brief 通过 DMA 发送数据
 * 
 * 半双工通信的发送实现：
 * 1. 检查总线是否空闲
 * 2. 标记总线为忙碌状态
 * 3. 停止当前 DMA 传输
 * 4. 设置 DMA 源地址和长度
 * 5. 拉高 DE 引脚（切换到发送模式）
 * 6. 清除 TC 中断标志
 * 7. 启用 USART DMA 请求和 DMA 通道
 * 
 * 发送完成后，USART TC 中断会自动拉低 DE 引脚并恢复接收模式。
 * 
 * @param data 待发送的数据指针
 * @param length 数据长度（字节）
 * 
 * @note 此函数不等待发送完成，发送通过 DMA 异步完成
 */
void bus_uart1_dma_send(unsigned char *data, uint16_t length)
{
    if (!bus_port_to_host.idle) return; // 总线正在使用

    bus_port_to_host.idle = false; // 标记为忙碌

    DMA1_Channel4->CFGR &= (uint16_t)(~DMA_CFGR1_EN); // 停止当前 DMA 传输

    DMA1_Channel4->MADDR = (uint32_t)data;  // 设置内存源地址
    DMA1_Channel4->CNTR  = length;           // 设置传输长度

    // DE = TX（拉高 PA12，切换到发送模式）
    GPIOA->BSHR = GPIO_Pin_12;

    // 清除 TC（传输完成）中断标志
    USART_ClearITPendingBit(USART1, USART_IT_TC);

    USART1->CTLR3 |= USART_DMAReq_Tx; // 启用 USART DMA 发送请求
    DMA1_Channel4->CFGR |= DMA_CFGR1_EN; // 启动 DMA 传输
}

/**
 * @brief USART1 中断服务程序
 * 
 * 处理两种中断源：
 * 
 * 1. RXNE（接收缓冲区非空）：
 *    - 读取接收到的字节
 *    - 仅在总线空闲时调用 IRQ 处理函数（避免在发送期间处理接收数据）
 * 
 * 2. TC（传输完成）：
 *    - DMA 发送完成，清除 TC 标志
 *    - 禁用 USART DMA 请求
 *    - 停止 DMA 通道
 *    - 拉低 DE 引脚（切换回接收模式）
 *    - 标记总线为空闲状态
 * 
 * @note 使用 WCH-Interrupt-fast 中断属性，减少中断响应延迟
 * @note 在 CH32V203 RISC-V MCU 上运行
 */
extern "C" void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        const uint8_t d = (uint8_t)USART_ReceiveData(USART1); // 读取接收数据（自动清除 RXNE）
        if (bus_port_to_host.idle) uart1_port_irq(d);         // 仅在空闲时处理
    }
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_TC);  // 清除 TC 标志
        USART1->CTLR3 &= ~USART_DMAReq_Tx;             // 禁用 USART DMA 请求
        DMA1_Channel4->CFGR &= (uint16_t)(~DMA_CFGR1_EN); // 停止 DMA 通道

        // DE = RX（拉低 PA12，切换回接收模式）
        GPIOA->BCR = GPIO_Pin_12;

        // 标记总线为空闲
        bus_port_to_host.idle = true;
    }
}
