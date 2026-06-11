#pragma once
#include "crc_bus.h"
#include <string.h>

/**
 * @brief 总线数据类型枚举
 * 
 * 根据数据包的魔数标识区分不同的总线协议类型。
 * 用于中断处理中的协议分发和主循环中的数据包类型判断。
 */
enum class _bus_data_type : uint8_t
{
    bambubus = 0x3D,  /**< BambuBus 协议（魔数 0x3D），用于打印机-AMS 通信 */
    ahub_bus = 0x33,  /**< AHUB 协议（魔数 0x33），用于集线器-AMS 通信 */
    none = 0x00       /**< 无有效数据包 */
};

/**
 * @brief 通知 BambuBus 模块更新心跳时间（从总线层 ISR 中调用）
 * 
 * 当收到有效数据包时，总线硬件层调用此函数通知上层协议栈刷新心跳计时器。
 */
void bambubus_heartbeat_seen_fast(void);

/**
 * @brief 总线端口数据处理类
 * 
 * 封装了总线串口通信的全部底层逻辑，包括：
 * - 双缓冲接收（ISR 写入缓冲区 A，主循环读取缓冲区 B）
 * - 数据包解析（魔数检测、长度计算、CRC8 校验）
 * - 心跳包快速处理（不经过完整协议栈）
 * - DMA 发送管理（双缓冲发送构建）
 * - 半双工收发控制（DE 引脚管理）
 * 
 * 中断处理流程（irq 函数）：
 * 1. 等待魔数 0x3D 或 0x33
 * 2. 根据帧标志判断短帧/长帧格式
 * 3. 在特定偏移读取长度字段
 * 4. 在 CRC8 偏移进行 CRC8 校验
 * 5. 快速检测心跳包（命令号 0x20），跳过后续数据
 * 6. 收到完整包后交换双缓冲指针
 * 
 * 发送流程：
 * 1. 主循环调用 tx_build_buf() 获取发送缓冲区
 * 2. 填充数据后设置 send_data_len
 * 3. send_package() 通过 DMA 发送
 * 
 * 线程安全：
 * - recv_data_len/bus_package_type/bus_recv_data_ptr 通过 IRQ 保护访问
 * - send_data_len 由主循环独占写入
 * - idle 标志控制半双工收发切换
 */
class _bus_port_deal // 中断数据处理
{
public:
    uint8_t send_data_buf[1280] __attribute__((aligned(4))); /**< 发送数据构建缓冲区（主循环使用） */

private:
    uint8_t tx_dma_buf[1280] __attribute__((aligned(4)));    /**< DMA 发送缓冲区（DMA 传输时使用） */
    uint8_t recv_data_buf[2][1280] __attribute__((aligned(4))); /**< 双接收缓冲区（ISR 和主循环交替使用） */
    uint8_t tx_build_sel = 0;    /**< 发送缓冲区选择标志（0=send_data_buf, 1=tx_dma_buf） */
    int _index = 0;              /**< 当前接收字节索引（ISR 内部状态） */
    int length = 999;            /**< 当前数据包预期总长度（ISR 内部计算） */
    uint8_t data_length_index = 0;  /**< 长度字段在包中的字节偏移（ISR 内部状态） */
    uint8_t data_CRC8_index = 0;    /**< CRC8 校验字段在包中的字节偏移（ISR 内部状态） */
    _bus_data_type irq_package_type = _bus_data_type::none; /**< ISR 中识别的协议类型 */
    uint8_t *bus_irq_data_ptr = recv_data_buf[0];  /**< ISR 当前写入的接收缓冲区指针 */
    int drop_bytes = 0;          /**< 需要丢弃的剩余字节数（心跳包快速处理时使用） */
    void (*port_send_datas)(uint8_t *data, uint16_t len); /**< 发送函数指针（指向 DMA 发送函数） */

public:
    uint8_t * volatile bus_recv_data_ptr = recv_data_buf[0]; /**< 主循环读取的接收缓冲区指针（volatile，ISR 可修改） */
    volatile int recv_data_len = 0;       /**< 主循环可读取的接收数据长度（0=无数据） */
    volatile int send_data_len = 0;       /**< 待发送的数据长度（0=无数据，非0时触发发送） */
    volatile _bus_data_type bus_package_type = _bus_data_type::none; /**< 主循环可读取的数据包类型 */
    volatile bool idle = true;            /**< 总线空闲标志（true=可发送，false=正在发送） */

    /**
     * @brief 获取当前发送构建缓冲区指针
     * 
     * 根据 tx_build_sel 标志返回合适的缓冲区：
     * - tx_build_sel=0: 返回 send_data_buf（主循环构建用）
     * - tx_build_sel=1: 返回 tx_dma_buf（DMA 发送用）
     * 
     * @return uint8_t* 指向可用发送缓冲区的指针
     */
    inline __attribute__((always_inline)) uint8_t* tx_build_buf()
    {
        return tx_build_sel ? tx_dma_buf : send_data_buf;
    }

    /**
     * @brief 初始化总线端口处理模块
     * 
     * 重置所有内部状态，设置发送函数指针。
     * 在系统启动时调用一次。
     * 
     * @param _port_send_datas 发送函数指针（指向 DMA 发送实现）
     */
    void init(void (*_port_send_datas)(uint8_t *data, uint16_t len))
    {
        _index = 0;
        length = 999;
        data_length_index = 0;
        data_CRC8_index = 0;
        irq_package_type = _bus_data_type::none;
        bus_irq_data_ptr = recv_data_buf[0];
        drop_bytes = 0;
        bus_recv_data_ptr = recv_data_buf[1]; // 主循环使用缓冲区 1
        idle = true;
        send_data_len = 0;
        recv_data_len = 0;
        tx_build_sel  = 0;
        port_send_datas = _port_send_datas;
    }

    /**
     * @brief UART 接收中断处理函数（逐字节调用）
     * 
     * 状态机实现的数据包接收和解析：
     * 
     * 状态 0（等待魔数）：
     *   - 检测 0x3D（BambuBus）或 0x33（AHUB）魔数
     *   - 识别后初始化帧解析参数
     * 
     * 状态 1（帧标志解析）：
     *   - 短帧（bit7=1）：长度字段在偏移 2，CRC8 在偏移 3
     *   - 长帧（bit7=0）：长度字段在偏移 4 或 5（BambuBus）或偏移 4（AHUB）
     * 
     * 长度字段解析：
     *   - BambuBus 短帧：1 字节长度
     *   - BambuBus 长帧：2 字节长度（小端序）
     *   - AHUB：1 字节长度 × 4 + 12
     * 
     * CRC8 校验：
     *   - 在指定偏移处验证 CRC8，失败则丢弃包
     * 
     * 心跳包快速处理：
     *   - BambuBus 短帧命令 0x20（心跳）：不等待完整包，直接丢弃剩余字节
     *   - 调用 bambubus_heartbeat_seen_fast() 更新心跳时间
     * 
     * 包完成处理：
     *   - 交换双缓冲指针（ISR 写入的缓冲区交给主循环读取）
     *   - 设置 recv_data_len 和 bus_package_type
     * 
     * @param data 接收到的单个字节（由 USART 中断传入）
     * 
     * @note 此函数在 USART1 中断上下文中调用，执行时间需尽可能短
     * @note 调用条件：bus_port_to_host.idle == true（总线空闲时才接收）
     */
    void irq(uint8_t data)
    {
        // 如果有需要丢弃的字节（心跳包快速处理后的剩余数据）
        if (drop_bytes > 0)
        {
            if (--drop_bytes == 0)
                bambubus_heartbeat_seen_fast(); // 丢弃完毕，更新心跳时间
            return;
        }

        const int BUF_SZ = (int)sizeof(recv_data_buf[0]);
        int idx = _index;

        // 状态 0：等待魔数
        if (idx == 0)
        {
            if (data == 0x3D || data == 0x33) // BambuBus 或 AHUB 魔数
            {
                bus_irq_data_ptr[0] = data;
                data_length_index = 4;      // 默认长帧长度字段偏移
                length = data_CRC8_index = 6; // 默认 CRC8 偏移
                _index = 1;
                irq_package_type = (_bus_data_type)data; // 记录协议类型
            }
            return;
        }

        // 边界检查
        if (idx < 0 || idx >= BUF_SZ)
        {
            _index = 0;
            return;
        }

        uint8_t *buf = bus_irq_data_ptr;
        buf[idx] = data; // 存储接收到的字节

        // 状态 1：帧标志解析
        if (idx == 1)
        {
            if (data & 0x80) // 短帧：bit7=1
            {
                data_length_index = 2;     // 短帧长度字段在偏移 2
                data_CRC8_index = 3;       // 短帧 CRC8 在偏移 3
            }
            else // 长帧：bit7=0
            {
                data_CRC8_index = 6;       // 长帧 CRC8 在偏移 6
                data_length_index = (irq_package_type == _bus_data_type::bambubus) ? 5 : 4;
                // BambuBus 长帧：长度字段在偏移 5
                // AHUB 长帧：长度字段在偏移 4
            }
        }

        // 长度字段解析
        if (idx == data_length_index)
        {
            if (irq_package_type == _bus_data_type::bambubus)
            {
                if (data_length_index == 2)
                    length = data;                    // 短帧：1 字节长度
                else
                    length = (int)buf[4] | ((int)data << 8); // 长帧：2 字节长度（小端序）
            }
            else if (irq_package_type == _bus_data_type::ahub_bus)
            {
                length = (((int)data) << 2) + 12;    // AHUB：长度 = 值×4 + 12
            }

            // 长度有效性检查
            if (length <= (int)data_CRC8_index || length > BUF_SZ)
            {
                _index = 0; // 长度无效，丢弃
                return;
            }
        }

        // CRC8 校验
        if (idx == data_CRC8_index)
        {
            if (data != bus_crc8(buf, (uint32_t)data_CRC8_index))
            {
                _index = 0; // CRC8 校验失败，丢弃
                return;
            }
        }

        // 心跳包快速处理：BambuBus 短帧命令 0x20
        if (irq_package_type == _bus_data_type::bambubus &&
            idx == 4 &&
            data_length_index == 2 &&
            length >= 6 &&
            buf[1] == 0xC5 &&
            data == 0x20) // 命令号 0x20 = 心跳
        {
            const int remain = length - 5; // 剩余需要丢弃的字节数
            _index = 0;

            if (remain > 0)
            {
                drop_bytes = remain; // 设置丢弃计数，后续字节将被跳过
            }
            else
            {
                bambubus_heartbeat_seen_fast(); // 没有剩余字节，直接更新心跳
            }
            return;
        }

        ++idx; // 前进到下一个字节

        // 包接收完成
        if (idx >= length)
        {
            _index = 0;

            // 只有当主循环尚未读取上一个包时才更新（避免覆盖）
            if (recv_data_len == 0)
            {
                // 交换双缓冲指针
                uint8_t *tmp = bus_recv_data_ptr;
                bus_recv_data_ptr = bus_irq_data_ptr; // 主循环读取 ISR 写入的缓冲区
                bus_irq_data_ptr = tmp;               // ISR 继续写入另一个缓冲区
                bus_package_type = irq_package_type;
                recv_data_len = length;
            }
            return;
        }

        _index = idx; // 保存状态，等待下一个字节
    }

    /**
     * @brief 发送已构建的数据包（通过 DMA）
     * 
     * 检查是否有待发送的数据，如果有且总线空闲：
     * 1. 获取当前发送缓冲区指针
     * 2. 切换发送缓冲区（双缓冲）
     * 3. 调用 DMA 发送函数
     * 4. 清空发送长度标志
     * 
     * @note 必须在主循环中调用，不能在中断中调用
     */
    void send_package()
    {
        const int len = send_data_len;
        if (len > 0 && len <= 1280)
        {
            if (!idle) return; // 总线正在发送，等待

            uint8_t *tx = tx_build_buf();
            tx_build_sel ^= 1; // 切换缓冲区（下次构建使用另一个）

            port_send_datas(tx, (uint16_t)len); // 启动 DMA 发送
            send_data_len = 0; // 清空标志
        }
    }

    /**
     * @brief 发送指定的数据（直接发送，不使用构建缓冲区）
     * 
     * 用于发送外部提供的数据包，直接通过 DMA 发送。
     * 
     * @param data 待发送的数据指针
     * @param len 数据长度（字节）
     */
    void send_package(uint8_t *data, uint16_t len)
    {
        if (len > 0 && len <= 1280)
        {
            if (!idle) return; // 总线正在发送，等待
            port_send_datas(data, len);
        }
    }
} __attribute__((aligned(4)));

/** @brief 全局总线端口实例，用于与主机（打印机/集线器）通信 */
extern _bus_port_deal bus_port_to_host;

/** @brief 初始化总线硬件（CRC 外设、UART、DMA 等） */
extern void bus_init();

/** @brief 无设备连接 */
#define host_device_type_none 0x0000
/** @brief AHUB 集线器设备类型 */
#define host_device_type_ahub 0x0001
/** @brief AMS 自动供料系统设备类型 */
#define host_device_type_ams 0x0700

/** @brief 当前连接的主机设备类型（用于长帧地址过滤） */
extern uint16_t bus_host_device_type;
