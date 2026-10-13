#pragma once
#include "crc_bus.h"
#include "hal/time_hw.h"
#include <string.h>

struct bus_rx_metrics
{
    volatile uint32_t rx_bytes;
    volatile uint32_t rx_frames_valid;
    volatile uint32_t rx_bad_length;
    volatile uint32_t rx_header_crc_error;
    volatile uint32_t rx_resync_bytes;
    volatile uint32_t rx_publish_drop;
    volatile uint32_t rx_dma_error;
    volatile uint32_t rx_usart_overrun;
    volatile uint32_t rx_dma_overrun;
    volatile uint32_t rx_dma_wrap;
    volatile uint32_t rx_dma_max_pending;
    volatile uint32_t rx_compat_copy;
};

enum class bus_tx_fault : uint8_t
{
    response_busy,
    response_missing,
    no_response_expected,
    invalid_length,
    dma_error,
    timeout,
};

struct bus_tx_metrics
{
    volatile uint32_t tx_started;
    volatile uint32_t tx_completed;
    volatile uint32_t tx_response_busy;
    volatile uint32_t tx_response_missing;
    volatile uint32_t tx_no_response_expected;
    volatile uint32_t tx_invalid_length;
    volatile uint32_t tx_dma_error;
    volatile uint32_t tx_timeout;
};

bool bus_uart1_rx_transport_quiet();
bool bus_uart1_reset_transport_quiescent();
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
    uint8_t tx_dma_buf[1280] __attribute__((aligned(4)));
    uint8_t recv_data_buf[3][1280] __attribute__((aligned(4)));
    uint8_t tx_build_sel = 0;
    uint8_t rx_read = 0, rx_write = 0, rx_count = 0;
    uint16_t rx_length[3] = {};
    _bus_data_type rx_type[3] = {};
    int _index = 0;
    int length = 999;
    uint8_t data_length_index = 0;
    uint8_t data_CRC8_index = 0;
    _bus_data_type irq_package_type = _bus_data_type::none;
    uint8_t *bus_irq_data_ptr = recv_data_buf[0];
    int heartbeat_drop_bytes = 0;
    bool (*port_send_datas)(uint8_t *data, uint16_t len);

public:
    bus_rx_metrics rx_metrics = {};
    bus_tx_metrics tx_metrics = {};
    uint8_t * volatile bus_recv_data_ptr = recv_data_buf[0]; /**< 主循环读取的接收缓冲区指针（volatile，ISR 可修改） */
    volatile int recv_data_len = 0;       /**< 主循环可读取的接收数据长度（0=无数据） */
    volatile int send_data_len = 0;       /**< 待发送的数据长度（0=无数据，非0时触发发送） */
    volatile _bus_data_type bus_package_type = _bus_data_type::none; /**< 主循环可读取的数据包类型 */
    volatile bool idle = true;            /**< 总线空闲标志（true=可发送，false=正在发送） */
    volatile uint32_t last_activity_tick = 0u;

    inline __attribute__((always_inline)) void note_activity()
    {
        last_activity_tick = time_ticks32();
    }

    inline __attribute__((always_inline)) void report_tx_fault(bus_tx_fault fault)
    {
        volatile uint32_t *counter = nullptr;
        switch (fault)
        {
        case bus_tx_fault::response_busy: counter = &tx_metrics.tx_response_busy; break;
        case bus_tx_fault::response_missing: counter = &tx_metrics.tx_response_missing; break;
        case bus_tx_fault::no_response_expected: counter = &tx_metrics.tx_no_response_expected; break;
        case bus_tx_fault::invalid_length: counter = &tx_metrics.tx_invalid_length; break;
        case bus_tx_fault::dma_error: counter = &tx_metrics.tx_dma_error; break;
        case bus_tx_fault::timeout: counter = &tx_metrics.tx_timeout; break;
        }
        if (counter != nullptr && *counter != 0xFFFFFFFFu) ++*counter;
    }

    inline __attribute__((always_inline)) bool quiet_for_us(uint32_t microseconds) const
    {
        if (!idle || send_data_len != 0 || recv_data_len != 0) return false;
        if (_index != 0 || heartbeat_drop_bytes != 0) return false;
        if (!bus_uart1_reset_transport_quiescent()) return false;
        return static_cast<uint32_t>(time_ticks32() - last_activity_tick) >=
               microseconds * time_hw_tpus;
    }

private:
    uint32_t send_not_before_tick = 0u;
    bool send_deferred = false;

    inline __attribute__((always_inline)) bool send_is_ready()
    {
        if (!send_deferred) return true;
        if (time_diff32(time_ticks32(), send_not_before_tick) < 0) return false;
        send_deferred = false;
        return true;
    }

public:
    inline __attribute__((always_inline)) void defer_send_us(uint32_t microseconds)
    {
        send_not_before_tick = time_ticks32() + microseconds * time_hw_tpus;
        send_deferred = true;
    }

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

    void init(bool (*_port_send_datas)(uint8_t *data, uint16_t len))
    {
        _index = 0;
        length = 999;
        data_length_index = 0;
        data_CRC8_index = 0;
        irq_package_type = _bus_data_type::none;
        bus_irq_data_ptr = recv_data_buf[0];
        heartbeat_drop_bytes = 0;
        bus_recv_data_ptr = recv_data_buf[0];
        rx_read = rx_write = rx_count = 0;
        idle = true;
        last_activity_tick = time_ticks32();
        send_deferred = false;
        send_data_len = 0;
        recv_data_len = 0;
        tx_build_sel  = 0;
        memset((void *)&rx_metrics, 0, sizeof(rx_metrics));
        memset((void *)&tx_metrics, 0, sizeof(tx_metrics));
        port_send_datas = _port_send_datas;
    }

    uint32_t rx_dropped = 0;
    volatile uint32_t tx_errors = 0;

    void reset_rx()
    {
        _index = 0;
        heartbeat_drop_bytes = 0;
        irq_package_type = _bus_data_type::none;
    }

    bool receiving() const { return _index != 0 || heartbeat_drop_bytes != 0; }

    void release_packet()
    {
        if (rx_count) {
            rx_read = (uint8_t)((rx_read + 1u) % 3u);
            --rx_count;
        }
        bus_recv_data_ptr = recv_data_buf[rx_read];
        bus_package_type = rx_count ? rx_type[rx_read] : _bus_data_type::none;
        recv_data_len = rx_count ? rx_length[rx_read] : 0;
    }

    void irq(uint8_t data)
    {
        note_activity();
        if (rx_metrics.rx_bytes != 0xFFFFFFFFu) ++rx_metrics.rx_bytes;
        // Heartbeat
        if (__builtin_expect(heartbeat_drop_bytes > 0, 0))
        {
            if (--heartbeat_drop_bytes == 0)
                bambubus_heartbeat_seen_fast();
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
            else if (rx_metrics.rx_resync_bytes != 0xFFFFFFFFu)
            {
                ++rx_metrics.rx_resync_bytes;
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

            const int minimum = irq_package_type == _bus_data_type::ahub_bus ? 12 :
                (data_length_index == 2 ? 7 : 15);
            if (length < minimum || length > BUF_SZ)
            {
                if (rx_metrics.rx_bad_length != 0xFFFFFFFFu) ++rx_metrics.rx_bad_length;
                _index = 0; // 长度无效，丢弃
                return;
            }
        }

        if (irq_package_type == _bus_data_type::bambubus &&
            idx == 4 && data_length_index == 2 && length >= 6 &&
            buf[1] == 0xC5u && data == 0x20u)
        {
            const int remain = length - 5;
            _index = 0;
            irq_package_type = _bus_data_type::none;
            if (remain > 0)
                heartbeat_drop_bytes = remain;
            else
                bambubus_heartbeat_seen_fast();
            return;
        }

        if (idx == data_CRC8_index)
        {
            if (data != bus_crc8(buf, (uint32_t)data_CRC8_index))
            {
                if (rx_metrics.rx_header_crc_error != 0xFFFFFFFFu)
                    ++rx_metrics.rx_header_crc_error;
                _index = 0; // CRC8 校验失败，丢弃
                return;
            }
        }

        ++idx;

        // 包接收完成
        if (idx >= length)
        {
            _index = 0;
            if (rx_metrics.rx_frames_valid != 0xFFFFFFFFu) ++rx_metrics.rx_frames_valid;

            if (rx_count < 2u)
            {
                rx_length[rx_write] = (uint16_t)length;
                rx_type[rx_write] = irq_package_type;
                if (rx_count == 0u) {
                    bus_recv_data_ptr = bus_irq_data_ptr;
                    bus_package_type = irq_package_type;
                    recv_data_len = length;
                }
                ++rx_count;
                rx_write = (uint8_t)((rx_write + 1u) % 3u);
                bus_irq_data_ptr = recv_data_buf[rx_write];
            }
            else
            {
                ++rx_dropped;
                if (rx_metrics.rx_publish_drop != 0xFFFFFFFFu) ++rx_metrics.rx_publish_drop;
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
        if (len == 0) return;
        if (len < 0 || len > 1280)
        {
            report_tx_fault(bus_tx_fault::invalid_length);
            send_data_len = 0;
            return;
        }
        {
            if (!idle || !send_is_ready()) return;

            uint8_t *tx = tx_build_buf();
            if (port_send_datas(tx, (uint16_t)len)) {
                tx_build_sel ^= 1;
                send_data_len = 0;
            }
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
void bus_rx_poll();
bool bus_background_ready();
void bus_shutdown();

/** @brief 无设备连接 */
#define host_device_type_none 0x0000
/** @brief AHUB 集线器设备类型 */
#define host_device_type_ahub 0x0001
/** @brief AMS 自动供料系统设备类型 */
#define host_device_type_ams 0x0700

/** @brief 当前连接的主机设备类型（用于长帧地址过滤） */
extern uint16_t bus_host_device_type;
