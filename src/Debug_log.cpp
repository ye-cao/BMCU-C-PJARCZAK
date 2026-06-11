/**
 * @file Debug_log.cpp
 * @brief 调试日志输出实现
 *
 * 使用 USART3 + DMA1_Channel2 实现低延迟的调试串口输出。
 * 硬件配置：
 * - TX: PB10（复用推挽输出，50MHz）
 * - RX: PB11（上拉输入）
 * - 波特率: 115200 bps（可配置）
 * - 数据格式: 9位数据位 + 停止位1 + 偶校验
 * - DMA: DMA1_Channel2 用于 USART3 发送
 *
 * 注意：当 Debug_log_on 未定义时，_write() 为空操作，
 *       但头文件中的 USART3 初始化代码仍会编译。
 *       如果 PB10/PB11 被用于其他功能（如 I2C2），
 *       需确保 Debug_log_on 未定义。
 */

#include "Debug_log.h"
#include <string.h>
#include <stddef.h>

#include "ch32v20x_rcc.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_usart.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_misc.h"

/* ===== USART3 接收中断处理 ===== */

/**
 * @brief USART3 中断服务程序
 *
 * 使用 WCH 快速中断属性（WCH-Interrupt-fast）。
 * 当接收到数据时（RXNE 中断），直接读取并丢弃。
 * 这里仅清除中断标志，不处理接收数据。
 */
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        (void)USART_ReceiveData(USART3);
    }
}

/* DMA 初始化结构体（全局复用） */
static DMA_InitTypeDef g_dma;
/* 调试串口初始化完成标志 */
static uint8_t g_dbg_inited = 0;

/**
 * @brief USART3 + DMA 发送初始化
 *
 * @param baudrate  波特率（如 115200）
 *
 * 完成以下配置：
 * 1. 使能 USART3、GPIOB、DMA1 时钟
 * 2. 配置 PB10 为复用推挽输出（TX）
 * 3. 配置 PB11 为上拉输入（RX）
 * 4. 配置 USART3 参数（9位数据、偶校验、无流控）
 * 5. 使能 USART3 接收中断
 * 6. 配置 DMA1_Channel2 为 USART3 发送通道
 */
static void Debug_uart3_dma_init(uint32_t baudrate)
{
    GPIO_InitTypeDef  gpio = {0};
    USART_InitTypeDef us   = {0};
    NVIC_InitTypeDef  nv   = {0};

    /* 使能外设时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* USART3: TX=PB10（复用推挽输出） */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    /* USART3: RX=PB11（上拉输入） */
    gpio.GPIO_Pin  = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    /* 配置 USART3 参数 */
    us.USART_BaudRate            = baudrate;
    us.USART_WordLength          = USART_WordLength_9b;   /* 9位数据（含校验位） */
    us.USART_StopBits            = USART_StopBits_1;      /* 1位停止位 */
    us.USART_Parity              = USART_Parity_Even;     /* 偶校验 */
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  /* 无流控 */
    us.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   /* 收发模式 */
    USART_Init(USART3, &us);

    /* 使能接收中断（防止接收缓冲区溢出） */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    /* 配置 NVIC 中断 */
    nv.NVIC_IRQChannel                   = USART3_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 1;
    nv.NVIC_IRQChannelSubPriority        = 1;
    nv.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nv);

    /* 配置 DMA1_Channel2 为 USART3 发送通道 */
    g_dma.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DATAR;  /* 外设地址：USART3 数据寄存器 */
    g_dma.DMA_MemoryBaseAddr     = (uint32_t)0;                /* 内存地址（发送时设置） */
    g_dma.DMA_DIR                = DMA_DIR_PeripheralDST;      /* 方向：内存→外设 */
    g_dma.DMA_Mode               = DMA_Mode_Normal;            /* 普通模式（非循环） */
    g_dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;  /* 外设地址不递增 */
    g_dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;       /* 内存地址递增 */
    g_dma.DMA_Priority           = DMA_Priority_Low;           /* 低优先级 */
    g_dma.DMA_M2M                = DMA_M2M_Disable;            /* 非内存到内存 */
    g_dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;    /* 内存数据宽度：字节 */
    g_dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /* 外设数据宽度：字节 */
    g_dma.DMA_BufferSize         = 0;

    /* 启用 USART3 */
    USART_Cmd(USART3, ENABLE);

    /* 初始化 DMA 通道（先禁用再复位） */
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_DeInit(DMA1_Channel2);
}

/**
 * @brief 初始化调试日志系统
 *
 * 仅初始化一次，后续调用直接返回。
 */
void Debug_log_init(void)
{
    if (g_dbg_inited) return;
    Debug_uart3_dma_init(Debug_log_baudrate);
    g_dbg_inited = 1;
}

/** 获取 64 位时间戳（当前未实现，返回 0） */
uint64_t Debug_log_count64(void) { return 0ULL; }

/** 输出时间信息（当前为空操作） */
void Debug_log_time(void) { }

/**
 * @brief 输出字符串
 *
 * @param data  以 '\0' 结尾的字符串指针
 *
 * 自动计算字符串长度，调用 Debug_log_write_num() 发送。
 */
void Debug_log_write(const void *data)
{
    int n = (int)strlen((const char*)data);
    Debug_log_write_num(data, n);
}

/**
 * @brief 输出指定长度的数据
 *
 * @param data  数据缓冲区指针
 * @param num   数据字节数
 *
 * 使用 DMA 发送数据：
 * 1. 等待上一次发送完成（TC 标志）
 * 2. 禁用并重新初始化 DMA 通道
 * 3. 设置新的内存地址和长度
 * 4. 清除 TC 标志（防止竞态条件）
 * 5. 启动 DMA 发送并使能 USART3 DMA 请求
 * 6. 等待发送完成
 */
void Debug_log_write_num(const void *data, int num)
{
    if (num <= 0) return;
    if (!g_dbg_inited) Debug_log_init();

    /* 等待上一次发送完成 */
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) { }

    /* 重新初始化 DMA 通道 */
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_DeInit(DMA1_Channel2);

    /* 设置新的发送缓冲区和长度 */
    g_dma.DMA_MemoryBaseAddr = (uint32_t)data;
    g_dma.DMA_BufferSize     = (uint16_t)num;
    DMA_Init(DMA1_Channel2, &g_dma);

    /* 清除 TC 标志，避免启动前误判为完成 */
    USART_ClearFlag(USART3, USART_FLAG_TC);

    /* 启动 DMA 发送 */
    DMA_Cmd(DMA1_Channel2, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

    /* 等待 DMA/UART 发送完成 */
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) { }
}

/**
 * @brief 标准库 _write() 函数重定向
 *
 * 将 printf() 的输出重定向到 USART3 调试串口。
 * 当 Debug_log_on 未定义时，此函数为空操作。
 */
__attribute__((used))
int _write(int fd, char *buf, int size)
{
#ifdef Debug_log_on
    (void)fd;
    Debug_log_write_num(buf, size);
#else
    (void)fd; (void)buf; (void)size;
#endif
    return size;
}

/**
 * @brief 标准库 _sbrk() 函数实现（堆内存管理）
 *
 * 为 newlib 提供堆空间管理。
 * 堆空间范围：_end 到 _heap_end（由链接脚本定义）。
 * 如果请求超出堆空间，返回 -1（失败）。
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
        return (void *)(-1);

    curbrk += incr;
    return curbrk - incr;
}
