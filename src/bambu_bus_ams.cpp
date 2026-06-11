#include "bambu_bus_ams.h"
#include <string.h>
#include "hal/irq_wch.h"
#include "ams.h"
#include "hal/time_hw.h"
#include "app_api.h"
#include "_bus_hardware.h"
#include "crc_bus.h"

/**
 * @brief AMS 编号到索引的映射表
 * 
 * 将逻辑 AMS 编号（0~3）映射到实际数组索引。
 * 用于在多个 AMS 设备间进行编号转换。
 */
uint8_t bambubus_ams_map[4] = {0, 1, 2, 3};

static void bambubus_build_static_serial(void);

/**
 * @brief 心跳超时截止时间（ticks 单位）
 * 
 * 记录下一次心跳必须到达的最晚时间。超过此时间未收到心跳则判定通信断开。
 * 初始值为 0，表示尚未开始计时。
 */
static uint32_t bambubus_heartbeat_deadline = 0u;

/**
 * @brief 快速更新心跳截止时间
 * 
 * 在收到有效数据包时调用，将心跳截止时间刷新为当前时间 + 1000ms。
 * 此函数需在中断上下文或主循环中安全调用。
 */
void bambubus_heartbeat_seen_fast(void)
{
    bambubus_heartbeat_deadline = time_ticks32() + ms_to_ticks32(1000u);
}

/**
 * @brief 校验数据包的 CRC16 完整性
 * 
 * 对数据包中 CRC16 字段之前的所有数据计算 CRC16，
 * 并与包末尾的 CRC16 字段进行比对。
 * 
 * @param data 指向数据包的指针
 * @param data_length 数据包总长度（字节）
 * @return true CRC 校验通过
 * @return false 数据包过短或 CRC 校验失败
 */
bool package_check_crc16(uint8_t *data, int data_length)
{
    if (data_length < 4) return false;

    const int crc_off = data_length - 2;                                    // CRC16 字段位于包尾倒数第 2 字节处
    const uint16_t num = bus_crc16(data, (uint32_t)crc_off);               // 计算 CRC16 校验值

    return (data[crc_off] == (num & 0xFFu)) &&
           (data[crc_off + 1] == ((num >> 8) & 0xFFu));                    // 低字节在前，高字节在后
}

/**
 * @brief 初始化 BambuBus 协议栈
 * 
 * 构建静态序列号（基于芯片唯一 ID），并重置心跳超时计数器。
 */
void bambubus_init()
{
    bambubus_build_static_serial();
    bambubus_heartbeat_deadline = 0u;
}

/**
 * @brief 为 BambuBus 数据包添加 CRC 校验
 * 
 * 根据数据包头标志位判断帧类型：
 * - 短帧头（bit7=1）：在偏移 3 处添加 CRC8
 * - 长帧头（bit7=0）：在偏移 6 处添加 CRC8
 * 
 * 然后在整个包（不含 CRC16 字段）上计算 CRC16 并追加到包尾。
 * 
 * @param data 指向待添加校验的数据包缓冲区
 * @param send_data_length 数据包总长度（含 CRC16 的 2 字节）
 */
void package_add_crc(uint8_t *data, int send_data_length)
{
    if (data[1] & 0x80) // 短帧：bit7=1
    {
        data[3] = bus_crc8(data, 3); // 短帧头校验：对前 3 字节计算 CRC8
    }
    else // 长帧：bit7=0
    {
        data[6] = bus_crc8(data, 6); // 长帧头校验：对前 6 字节计算 CRC8
    }
    send_data_length -= 2;                                              // CRC16 字段之前的长度
    uint16_t num = bus_crc16(data, (uint32_t)send_data_length);        // 计算 CRC16 校验值
    data[(send_data_length)] = num & 0xFF;                              // CRC16 低字节
    data[(send_data_length + 1)] = num >> 8;                            // CRC16 高字节
}

/**
 * @brief BambuBus 长帧数据包结构体
 * 
 * 用于打印机与 AMS 之间的长帧通信（如耗材信息读写、版本查询等）。
 * 长帧通过地址寻址方式通信，支持任意源地址到目标地址的数据传输。
 */
struct bambubus_long_packge_data
{
    uint16_t package_number;    /**< 数据包序列号，用于请求-响应匹配 */
    uint16_t package_length;    /**< 数据包总长度（字节） */
    uint8_t crc8;              /**< 帧头 CRC8 校验值 */
    uint16_t target_address;   /**< 目标设备地址（如 AMS 设备地址 0x0700） */
    uint16_t source_address;   /**< 源设备地址（如打印机地址） */
    uint16_t type;             /**< 命令类型码（如 0x211=读耗材信息，0x103=版本查询） */
    uint8_t *datas;            /**< 指向有效载荷数据的指针 */
    uint16_t data_length;      /**< 有效载荷数据长度（字节） */
} __attribute__((packed));

/**
 * @brief 构建并发送 BambuBus 长帧数据包
 * 
 * 将长帧数据结构体序列化为可发送的字节流：
 * 1. 设置魔数 0x3D 和帧标志 0x00
 * 2. 复制帧头字段（11 字节）和有效载荷数据
 * 3. 添加 CRC8 和 CRC16 校验
 * 4. 设置发送长度，触发后续发送
 * 
 * @param data 指向待发送的长帧数据结构体
 */
void bambubus_long_package_get(bambubus_long_packge_data *data)
{
    if (bus_port_to_host.send_data_len != 0) return; // 上一个包尚未发送完毕
    uint8_t* out = bus_port_to_host.tx_build_buf();

    out[0] = 0x3D; // BambuBus 魔数标识
    out[1] = 0x00; // 长帧标志

    data->package_length = data->data_length + 15;    // 长帧总长 = 帧头11字节 + 载荷 + CRC16 2字节 + 2字节头部偏移
    memcpy(out + 2,  data,        11);                // 复制帧头字段（package_number ~ type）
    memcpy(out + 13, data->datas, data->data_length); // 复制有效载荷数据

    package_add_crc(out, data->data_length + 15);     // 添加 CRC 校验
    bus_port_to_host.send_data_len = data->data_length + 15; // 设置发送长度
}

/**
 * @brief 解析接收到的 BambuBus 长帧数据包
 * 
 * 将接收到的原始字节流解析为长帧数据结构体。
 * 从偏移 2 开始提取帧头字段（11 字节），并将数据指针指向偏移 13 处的有效载荷。
 * 
 * @param buf 接收到的原始数据缓冲区
 * @param data_length 数据总长度
 * @param data 输出：解析后的长帧数据结构体
 */
void bambubus_long_package_analysis(uint8_t *buf, int data_length, bambubus_long_packge_data *data)
{
    if (data_length < 15) {       // 长帧最小长度为 15 字节（2字节头 + 11字节帧头 + 2字节CRC16）
        memset(data, 0, sizeof(*data));
        return;
    }
    memcpy(data, buf + 2, 11);    // 从偏移 2 开始复制帧头字段（11字节）
    data->datas = buf + 13;       // 有效载荷起始于偏移 13
    data->data_length = data_length - 15; // 载荷长度 = 总长 - 帧头(11) - 头部(2) - CRC16(2)
}

/** @brief 打印机发来的长帧数据包全局解析结果 */
bambubus_long_packge_data printer_data_long;

/** @brief 当前在线检测前缀值（0x0C 或 0x0A） */
static uint8_t online_detect_prefix_now = 0x0Cu;
/** @brief 标记 AMS 是否已成功注册到打印机 */
static bool have_registered = false;
/** @brief 在线检测阶段（0=初始，1=第一次检测，2=重复检测，3=已注册） */
static uint8_t online_detect_phase = 0u;

/**
 * @brief 重置在线检测状态
 * 
 * 将在线检测相关的所有状态变量恢复为初始值。
 * 在 AMS 下线或重新初始化时调用。
 */
static inline void online_detect_reset(void)
{
    have_registered = false;
    online_detect_prefix_now = 0x0Cu;
    online_detect_phase = 0u;
}

/**
 * @brief 获取接收到的 BambuBus 数据包类型
 * 
 * 解析原始数据包，根据魔数、帧标志和命令号判断数据包类型。
 * 支持两种帧格式：
 * - 短帧（buf[1] == 0xC5）：通过 buf[4] 命令号区分
 * - 长帧（buf[1] == 0x05 或 0x04）：通过帧头中的 type 字段区分
 * 
 * @param buf 接收到的原始数据
 * @param length 数据长度
 * @return bambubus_package_type 解析出的数据包类型
 */
bambubus_package_type get_packge_type(unsigned char *buf, int length)
{
    if (length < 6) return bambubus_package_type::none;
    if (buf[0] != 0x3D) return bambubus_package_type::none;         // 魔数校验
    if (!package_check_crc16(buf, length)) return bambubus_package_type::none; // CRC16 校验

    if (buf[1] == 0xC5) // 短帧标志
    {
        switch (buf[4]) // 命令号位于偏移 4
        {
        case 0x03:
            return bambubus_package_type::filament_motion_short;    // 短帧耗材运动命令
        case 0x04:
            return bambubus_package_type::filament_motion_long;     // 长帧耗材状态命令
        case 0x05:
            return bambubus_package_type::online_detect;            // 在线检测
        case 0x06:
            return bambubus_package_type::REQx6;                    // 请求 0x06
        case 0x07:
            return bambubus_package_type::NFC_detect;               // NFC 检测
        case 0x08:
            return bambubus_package_type::set_filament_info;        // 设置耗材信息
        case 0x20:
            return bambubus_package_type::heartbeat;                // 心跳包
        default:
            return bambubus_package_type::ETC;
        }
    }
    else if ((buf[1] == 0x05) || (buf[1] == 0x04)) // 长帧标志
    {
        if (length < 15) return bambubus_package_type::none;
        bambubus_long_package_analysis(buf, length, &printer_data_long);
        if (printer_data_long.target_address != host_device_type_ams) // 目标地址必须是 AMS
        {
            return bambubus_package_type::none;
        }

        switch (printer_data_long.type) // 命令类型码
        {
        case 0x21A:
            return bambubus_package_type::MC_online;                // 主控在线确认
        case 0x211:
            return bambubus_package_type::read_filament_info;       // 读取耗材信息
        case 0x218:
            return bambubus_package_type::set_filament_info_type2;  // 设置耗材信息（长帧版本）
        case 0x103:
            return bambubus_package_type::version;                  // 版本查询
        case 0x402:
            return bambubus_package_type::serial_number;            // 序列号查询
        default:
            return bambubus_package_type::ETC;
        }
    }
    return bambubus_package_type::none;
}

/** @brief 数据包序列号计数器，循环 0~7 */
uint8_t package_num = 0;

/**
 * @brief 获取各耗材通道的在线和运动状态标志
 * 
 * 遍历 4 个耗材通道，为每个通道编码 2 位状态：
 * - bit0: 耗材是否在线
 * - bit1: 耗材是否在运动中（非空闲）
 * 
 * @param ams 指向 AMS 数据结构体
 * @return uint8_t 8 位状态标志，每 2 位对应一个通道（bit[1:0]=通道0, bit[3:2]=通道1, ...）
 */
uint8_t get_filament_left_char(const _ams *ams)
{
    uint8_t data = 0u;

    for (uint8_t i = 0; i < 4u; i++)
    {
        if (!ams->filament[i].online) continue;    // 跳过离线的耗材

        data |= (uint8_t)(1u << (i << 1));         // 设置在线标志位（bit0, bit2, bit4, bit6）
        if (ams->filament[i].motion != _filament_motion::idle)
            data |= (uint8_t)(2u << (i << 1));     // 设置运动标志位（bit1, bit3, bit5, bit7）
    }

    return data;
}

/**
 * @brief 各通道从送丝到使用状态的超时计时器（ticks 单位）
 * 
 * 当耗材从"送丝"(send_out) 状态切换到"使用"(on_use) 状态时，
 * 如果在超时时间内未完成切换，则强制进入使用状态。
 * 用于处理送丝后打印机延迟进入使用状态的情况。
 */
static uint32_t time_sendout_onuse_ticks[4] = {};

/** @brief 上一次 before_on_use 事件的 motion_flag 值（0x7F 或 0xA5） */
static uint8_t last_before_on_use_motion_flag = 0x00;
/** @brief on_use 状态重复计数器，用于首次使用时的特殊压力控制 */
static uint8_t count_on_use = 0u;

/**
 * @brief 设置耗材运动状态
 * 
 * 根据打印机发送的命令参数，更新指定 AMS 中指定耗材通道的运动状态。
 * 实现完整的状态机逻辑，包括：
 * - send_out（送丝）：耗材从料架送出
 * - before_on_use（使用前准备）：耗材到达打印头前的准备阶段
 * - on_use（使用中）：耗材正在被打印头使用
 * - stop_on_use（使用中停止）：使用状态下的暂停
 * - before_pull_back（回退前准备）：准备将耗材回退
 * - pull_back（回退）：将耗材回退到料架
 * 
 * 同时管理压力参数和耗材使用标志，确保打印机获得正确的反馈。
 * 
 * @param read_num 耗材通道号（0~3），0xFF 表示全局命令
 * @param statu_flags 状态标志（0x01=就绪, 0x03=送丝, 0x07=使用, 0x09=准备）
 * @param fliment_motion_flag 运动标志（0x00=停止, 0x3F=回退, 0x7F=正常, 0xA5=特殊）
 * @param ams_num AMS 编号
 * @return true 命令已处理
 * @return false 命令被拒绝（AMS 编号不匹配）
 */
bool set_motion(unsigned char read_num, unsigned char statu_flags, unsigned char fliment_motion_flag, uint8_t ams_num)
{
    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    if (ams_num != fixed_ams_num) return false; // AMS 编号不匹配，拒绝命令

    _ams *ams_ptr = &ams[bambubus_ams_map[fixed_ams_num]];

    if (read_num < 4) // 特定通道命令
    {
        const uint8_t ch = (uint8_t)read_num;

        // 解析状态标志组合，判断具体命令类型
        const bool is_send_out      = ((statu_flags == 0x03) && (fliment_motion_flag == 0x00));   // 送丝命令
        const bool is_before_on_use = ((statu_flags == 0x09) && ((fliment_motion_flag == 0x7F) || (fliment_motion_flag == 0xA5))); // 使用前准备
        const bool is_stop_on_use   = ((statu_flags == 0x07) && (fliment_motion_flag == 0x00));   // 使用中停止
        const bool is_on_use        = ((statu_flags == 0x07) && (fliment_motion_flag == 0x7F));   // 使用中
        const bool is_before_pullb  = ((statu_flags == 0x09) && (fliment_motion_flag == 0x3F));   // 回退前准备

        uint32_t &t_sendout_onuse = time_sendout_onuse_ticks[ch];

        const uint8_t loaded = ams_state_get_loaded();    // 获取当前已装载的通道号
        const bool allow_any = (loaded == 0xFFu) || (loaded == ch); // 允许任何操作：未装载或当前通道
        const bool allow_stop = (loaded == ch);           // 允许停止：必须是当前已装载的通道

        // 综合判断是否接受该命令
        const bool accept =
            (is_send_out) ||
            (is_before_on_use && allow_any) ||
            (is_on_use        && allow_any) ||
            (is_stop_on_use   && allow_stop) ||
            (is_before_pullb  && allow_stop);

        if (accept)
        {
            // 如果切换了通道，需要将旧通道恢复为空闲状态
            if (ams_ptr->now_filament_num != ch)
            {
                if (ams_ptr->now_filament_num < 4)
                {
                    const uint8_t prev = ams_ptr->now_filament_num;
                    ams_ptr->filament[prev].motion = _filament_motion::idle;  // 旧通道恢复空闲
                    ams_ptr->filament_use_flag = 0x00;
                    ams_ptr->pressure = 0xF9C6;                              // 空闲状态压力值
                    time_sendout_onuse_ticks[prev] = 0u;
                }
                bus_now_ams_num = bambubus_ams_map[fixed_ams_num];
                ams_ptr->now_filament_num = ch; // 切换到新通道
            }
        }

        if (is_send_out)
        {
            // 送丝状态：耗材从料架送出
            t_sendout_onuse = 0u;
            count_on_use = 0u;

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            // 如果之前有其他通道在使用中，将其标记为未装载
            if (prev != _filament_motion::send_out && ams_state_get_loaded() != 0xFFu)
                ams_state_set_unloaded(0xFFu);

            ams_ptr->filament[ch].motion = _filament_motion::send_out;
            ams_ptr->filament_use_flag = 0x02;    // 送丝使用标志
            ams_ptr->pressure = 0x4700;           // 送丝状态压力值
        }
        else if (is_before_on_use)
        {
            // 使用前准备：耗材到达打印头前的准备阶段
            t_sendout_onuse = 0u;
            count_on_use = 0u;

            if (!allow_any) return true; // 不允许操作，直接返回

            last_before_on_use_motion_flag = fliment_motion_flag; // 记录运动标志

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            ams_ptr->filament[ch].motion = _filament_motion::before_on_use;
            ams_ptr->filament_use_flag = 0x04;    // 使用中标志

            if (fliment_motion_flag == 0x7F)
            {
                ams_ptr->pressure = 0x1E34;       // 正常准备压力
            }
            else
            {
                // 从送丝状态转来使用较高压力，否则使用默认压力
                ams_ptr->pressure = (prev == _filament_motion::send_out) ? 0x4700 : 0x2B00;
            }

            ams_state_set_loaded(ch);             // 标记该通道已装载
        }
        else if (is_stop_on_use)
        {
            // 使用中停止：暂停当前耗材输送
            t_sendout_onuse = 0u;

            if (!allow_stop) return true;         // 只有当前装载通道才能停止

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            // 只有在使用中或准备中才能切换到停止状态
            if (prev == _filament_motion::on_use ||
                prev == _filament_motion::before_on_use)
            {
                ams_ptr->filament[ch].motion = _filament_motion::stop_on_use;
            }

            ams_ptr->filament_use_flag = 0x04;
            ams_ptr->pressure = 0x2B00;

            // 如果之前是 0x7F 准备状态，使用特殊压力值
            if (prev == _filament_motion::before_on_use && last_before_on_use_motion_flag == 0x7F)
            {
                ams_ptr->pressure = 0x1E34;
            }
        }
        else if (is_on_use)
        {
            // 使用中：耗材正在被打印头消耗
            if (!allow_any) return true;

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            if (prev == _filament_motion::send_out)
            {
                // 从送丝状态切换到使用状态，需要等待送丝完成
                if (time_hw_tpms != 0u)
                {
                    const uint32_t now = time_ticks32();
                    if (t_sendout_onuse == 0u) t_sendout_onuse = now; // 开始计时

                    const uint32_t dt = (uint32_t)(now - t_sendout_onuse);
                    const uint32_t lim = 15000u * (uint32_t)time_hw_tpms; // 超时限制

                    if (dt < lim)
                    {
                        // 送丝尚未完成，保持等待状态
                        ams_ptr->filament_use_flag = 0x04;
                        ams_ptr->pressure = 0x2B00;
                        return true;
                    }

                    t_sendout_onuse = 0u; // 超时，允许进入使用状态
                }
                else
                {
                    // 无法计算超时，保持等待
                    ams_ptr->filament_use_flag = 0x04;
                    ams_ptr->pressure = 0x2B00;
                    return true;
                }
            }

            t_sendout_onuse = 0u;

            ams_ptr->filament[ch].motion = _filament_motion::on_use;
            ams_ptr->filament_use_flag = 0x04;

            // 保持特殊压力值 0xF06F，否则使用默认压力
            if (ams_ptr->pressure != 0xF06Fu) ams_ptr->pressure = 0x2B00;

            // 首次使用时的特殊压力控制（0x7F 准备状态后前 5 次）
            if (last_before_on_use_motion_flag == 0x7F && count_on_use < 5)
            {
                count_on_use++;
                ams_ptr->pressure = 0x1E34;
            }

            ams_state_set_loaded(ch);             // 标记该通道已装载
        }
        else if (is_before_pullb)
        {
            // 回退前准备：准备将耗材从打印头回退
            t_sendout_onuse = 0u;

            if (!allow_stop) return true;

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            // 只有在使用中、准备中或停止状态下才能回退
            if (prev == _filament_motion::on_use ||
                prev == _filament_motion::before_on_use ||
                prev == _filament_motion::stop_on_use)
            {
                ams_ptr->filament[ch].motion = _filament_motion::before_pull_back;
            }

            ams_ptr->filament_use_flag = 0x04;
            ams_ptr->pressure = 0x2B00;

            ams_state_set_unloaded(ch);           // 标记该通道已卸载
        }
        else if (statu_flags == 0x09)
        {
            // 其他 0x09 状态：仅更新标志和压力
            t_sendout_onuse = 0u;
            ams_ptr->filament_use_flag = 0x04;
            ams_ptr->pressure = 0x2B00;
        }
    }
    else if (read_num == 0xFF) // 全局命令（影响所有通道）
    {
        if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00))
        {
            // 全局送丝命令：将当前使用通道切换到回退状态
            if (ams_ptr->now_filament_num < 4)
            {
                const uint8_t ch = ams_ptr->now_filament_num;
                const _filament_motion m = ams_ptr->filament[ch].motion;

                time_sendout_onuse_ticks[ch] = 0u;

                // 如果当前通道在使用中，切换到回退状态
                if (m == _filament_motion::before_pull_back ||
                    m == _filament_motion::on_use ||
                    m == _filament_motion::before_on_use ||
                    m == _filament_motion::stop_on_use)
                {
                    ams_ptr->filament[ch].motion = _filament_motion::pull_back;
                    ams_ptr->filament_use_flag = 0x02;
                }

                ams_ptr->pressure = 0x4700;
                ams_state_set_unloaded(ch);
            }
        }
        else if (statu_flags == 0x01)
        {
            // 状态 0x01：通道就绪，标记为未装载
            const uint8_t ch = ams_ptr->now_filament_num;
            if (ch < 4 && ams_ptr->filament_use_flag != 0x04)
                ams_state_set_unloaded(ch);
        }
        else
        {
            // 其他全局命令：如果当前通道在使用中则拒绝，否则重置所有通道
            if (ams_ptr->now_filament_num < 4)
            {
                const uint8_t ch = ams_ptr->now_filament_num;
                const _filament_motion m = ams_ptr->filament[ch].motion;

                if (m == _filament_motion::on_use ||
                    m == _filament_motion::before_on_use ||
                    m == _filament_motion::stop_on_use)
                {
                    return true; // 使用中不允许重置
                }
            }

            // 重置所有通道为空闲状态
            for (uint8_t i = 0; i < 4; i++)
            {
                ams_ptr->filament[i].motion = _filament_motion::idle;
                time_sendout_onuse_ticks[i] = 0u;
            }

            ams_ptr->filament_use_flag = 0x00;
            ams_ptr->pressure = 0xF9C6;           // 空闲状态压力值
            ams_ptr->now_filament_num = 0xFF;      // 无通道在使用
            ams_state_set_unloaded(0xFFu);
        }
    }

    return true;
}

/**
 * @brief 打印机发来的短帧运动命令包结构体
 * 
 * 打印机通过此命令请求 AMS 报告耗材运动状态。
 * 短帧格式：魔数(1) + 标志(1) + 长度(1) + CRC8(1) + 命令(1) + 数据(6) + CRC16(2) = 12字节
 */
struct bambubus_printer_motion_package_struct
{
    uint8_t magic_byte;         /**< 魔数 0x3D */
    uint8_t flag;              /**< 帧标志 0xC5（短帧） */
    uint8_t length;            /**< 数据包长度 */
    uint8_t crc8;             /**< 帧头 CRC8 校验 */
    uint8_t command;          /**< 命令号 0x03 */
    uint8_t ams_num;          /**< AMS 编号 */
    uint8_t statu_flag;       /**< 状态标志 */
    uint8_t filamnet_channel; /**< 耗材通道号 */
    uint8_t motion_flag;      /**< 运动标志 */
    uint8_t unknow;           /**< 未知字段 */
    uint16_t crc16;           /**< CRC16 校验 */
} __attribute__((packed));

/**
 * @brief AMS 回复的短帧运动状态包结构体
 * 
 * AMS 在收到打印机的运动命令后，通过此格式回复当前状态。
 * 包含耗材使用标志、当前通道、计米值、压力值等信息。
 */
struct bambubus_ams_motion_package_struct
{
    uint8_t magic_byte = 0x3D;          /**< 魔数 0x3D */
    uint8_t flag = 0xC0;               /**< 帧标志 0xC0（短帧回复） */
    uint8_t length = 0x2C;             /**< 数据包长度（44字节） */
    uint8_t crc8;                      /**< 帧头 CRC8 校验 */
    uint8_t command = 0x03;            /**< 命令号 0x03 */
    uint8_t ams_num = 0;               /**< AMS 编号 */
    uint8_t unknow1 = 0x00;           /**< 未知字段1 */
    uint8_t filament_use_flag = 0x00;  /**< 耗材使用标志（0x00=空闲, 0x02=送丝, 0x04=使用中） */
    uint8_t filament_channel = 0x00;   /**< 当前正在使用的耗材通道号 */
    float meters = 0;                  /**< 计米值（耗材消耗长度，单位：米） */
    uint16_t pressure = 0;             /**< 压力传感器值 */
    uint16_t unknow2 = 0xFFFF;        /**< 未知字段2 */
    uint8_t unknow3[12];              /**< 未知字段3（12字节） */
    uint8_t filament_stu_flag = 0x00;  /**< 各耗材通道在线/运动状态标志 */
    uint32_t last1 = 0xFFFFFFFF;      /**< 末尾填充字段1 */
    uint32_t last2 = 0x01010101;      /**< 末尾填充字段2 */
    uint8_t filament_channel_2 = 0x00; /**< 重复的通道号字段 */
    uint8_t last4 = 0x00;             /**< 末尾填充字段3 */
    uint16_t last5 = 0x0000;          /**< 末尾填充字段4 */
    uint16_t crc16;                   /**< CRC16 校验 */
} __attribute__((packed));

/**
 * @brief AMS 短帧运动状态包的默认初始化数据
 * 
 * 作为模板使用，在构建回复包时先用此数据填充，再根据实际情况修改字段。
 */
static const bambubus_ams_motion_package_struct _bambubus_ams_motion_package_struct_init_data = {
    0x3D,       // magic_byte - 魔数标识
    0xC0,       // flag - 短帧回复标志
    0x2C,       // length - 包长度
    0x00,       // crc8 - CRC8 校验（运行时计算）
    0x03,       // command - 命令号
    0x00,       // ams_num - AMS 编号（运行时设置）
    0x00,       // unknow1
    0x00,       // filament_use_flag - 使用标志
    0x00,       // filament_channel - 通道号
    0.0f,       // meters - 计米值
    0x0000,     // pressure - 压力值
    0xFFFF,     // unknow2
    {           // unknow3[12]
        0x36, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xF7, 0xFF, 0x00, 0x00, 0x27, 0x00
    },
    0x00,           // filament_stu_flag
    0xF6F2F4FB,    // last1
    0xB1B4B7B5,    // last2
    0x00,           // filament_channel_2
    0x00,           // last4
    0x0000,         // last5
    0x0000          // crc16
};

/**
 * @brief 使用前准备阶段（0x7F）的嗅探数据行
 * 
 * 通过抓包分析获得的打印机期望的回复序列。
 * 当耗材处于 before_on_use 状态且 motion_flag 为 0x7F 时，
 * AMS 需要按照此序列逐步回复特定的压力和参数值，模拟真实 AMS 的行为。
 */
struct before_on_use_sniff_row
{
    uint16_t pressure;      /**< 压力传感器值 */
    uint16_t unknow2;       /**< 未知字段2 */
    uint8_t unknow3[12];   /**< 未知字段3（12字节） */
    uint32_t last1;         /**< 末尾填充字段1 */
    uint32_t last2;         /**< 末尾填充字段2 */
};

/**
 * @brief 使用前准备阶段（0x7F）的嗅探数据表
 * 
 * 包含 44 帧从真实 AMS 抓包获得的数据。当耗材进入 before_on_use 状态时，
 * AMS 需要按照此表中的顺序逐步回复，以满足打印机的协议要求。
 * 每一帧对应一个特定的压力和参数组合。
 */
static const before_on_use_sniff_row before_on_use_sniff_7f_rows[] = {
    { 0x403Du, 0xFFF8u, { 0x36, 0x00, 0x00, 0x00, 0xF7, 0xFF, 0xF6, 0xFF, 0x00, 0x00, 0xD9, 0xFF }, 0xF3F4FA57u, 0xB4B5F0F7u }, // 帧 55848
    { 0x3FE7u, 0xFEC9u, { 0x81, 0xE9, 0xCE, 0xF6, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0xD9, 0xFF }, 0xF3F4FA57u, 0xB4B5F0F7u }, // 帧 55854
    { 0x4006u, 0xF8D8u, { 0xBC, 0xEA, 0x56, 0xE8, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x2A, 0x06 }, 0xF3F4FA57u, 0xB4B5EDF7u }, // 帧 55859
    { 0x3D8Eu, 0xF798u, { 0x6C, 0xEB, 0xB5, 0xE4, 0xF5, 0xFF, 0xF4, 0xFF, 0x00, 0x00, 0x0B, 0xFB }, 0xF3F4FA57u, 0xB4B5ECF7u }, // 帧 55861
    { 0x1BC9u, 0x0168u, { 0xDE, 0x00, 0x1B, 0x04, 0xF5, 0xFF, 0xF4, 0xFF, 0x00, 0x00, 0xE7, 0xFE }, 0xF3F4FA57u, 0xB4B5ECF6u }, // 帧 55872
    { 0x1B8Du, 0x024Au, { 0xDE, 0x01, 0x59, 0x06, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x1A, 0xFF }, 0xF3F4FA57u, 0xB4B5ECF6u }, // 帧 55874
    { 0x1B71u, 0x0368u, { 0x66, 0x02, 0xBE, 0x07, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x36, 0xFF }, 0xF3F4FA57u, 0xB4B5ECF6u }, // 帧 55876
    { 0x1B99u, 0x0487u, { 0x2B, 0x02, 0x61, 0x09, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0xAE, 0x00 }, 0xF3F4FA57u, 0xB4B5ECF7u }, // 帧 55878
    { 0x1C11u, 0x07F7u, { 0x53, 0x02, 0x14, 0x0C, 0xF4, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x98, 0x00 }, 0xF3F4FA57u, 0xB4B5ECF7u }, // 帧 55884
    { 0x1E4Fu, 0x056Au, { 0x57, 0x01, 0x39, 0x08, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0xAE, 0x00 }, 0xF3F4FA57u, 0xB4B5ECF7u }, // 帧 55893
    { 0x1DB5u, 0x06A0u, { 0xA3, 0x06, 0xE0, 0x08, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x33, 0x02 }, 0xF3F4FA57u, 0xB4B5EDF6u }, // 帧 55907
    { 0x1D86u, 0x054Cu, { 0x7E, 0x06, 0x69, 0x07, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x86, 0x01 }, 0xF3F4FA57u, 0xB4B5EDF6u }, // 帧 55909
    { 0x1D42u, 0x04F8u, { 0x00, 0x04, 0xDE, 0x07, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x3D, 0x01 }, 0xF3F4FA57u, 0xB4B5EDF6u }, // 帧 55911
    { 0x1B83u, 0x0A32u, { 0x2D, 0x01, 0x73, 0x0F, 0xF4, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xA3, 0x00 }, 0xF3F4FB57u, 0xB4B5EDF6u }, // 帧 55918
    { 0x1C70u, 0x058Au, { 0xFE, 0x01, 0x47, 0x08, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0xD3, 0x00 }, 0xF3F4FB57u, 0xB4B5EEF6u }, // 帧 55929
    { 0x1BE9u, 0x0980u, { 0x41, 0x01, 0xA6, 0x0D, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x9E, 0x00 }, 0xF3F4FB57u, 0xB4B5EEF7u }, // 帧 55934
    { 0x1C04u, 0x0A51u, { 0x52, 0x05, 0xEE, 0x0E, 0xF4, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x33, 0x02 }, 0xF3F4FB57u, 0xB4B5EFF7u }, // 帧 55940
    { 0x1D9Au, 0x05B7u, { 0xDC, 0x03, 0x74, 0x08, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xD3, 0x00 }, 0xF3F4FB57u, 0xB4B5EFF7u }, // 帧 55946
    { 0x1D3Du, 0x0583u, { 0x86, 0x02, 0x4E, 0x08, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0xB5, 0x00 }, 0xF3F4FB57u, 0xB4B5F0F6u }, // 帧 55948
    { 0x1B9Eu, 0x09AAu, { 0x39, 0x01, 0x0B, 0x0F, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x97, 0x00 }, 0xF3F4FB57u, 0xB4B5F0F7u }, // 帧 55953
    { 0x1C75u, 0x09C6u, { 0xCE, 0x05, 0x64, 0x0D, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xC3, 0xFE }, 0xF3F4FB57u, 0xB4B5F0F6u }, // 帧 55959
    { 0x1D12u, 0x0642u, { 0x52, 0x02, 0xF1, 0x08, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x6A, 0x01 }, 0xF3F4FB57u, 0xB4B5F0F7u }, // 帧 55965
    { 0x1C4Cu, 0x09FFu, { 0x4D, 0x05, 0x52, 0x0F, 0xF4, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x9F, 0x00 }, 0xF3F4FB57u, 0xB4B5F0F7u }, // 帧 55974
    { 0x1E28u, 0x05C0u, { 0x82, 0x06, 0xE3, 0x07, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x9F, 0x00 }, 0xF3F4FB57u, 0xB4B5F0F6u }, // 帧 55978
    { 0x1D59u, 0x057Cu, { 0x6A, 0x02, 0x31, 0x08, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x52, 0x01 }, 0xF3F4FB57u, 0xB4B5F0F7u }, // 帧 55981
    { 0x1CD9u, 0x0852u, { 0x8E, 0x04, 0x8C, 0x0A, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xFB, 0x01 }, 0xF3F4FB57u, 0xB4B5F0F6u }, // 帧 55996
    { 0x1CD7u, 0x070Cu, { 0x24, 0x03, 0x92, 0x09, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x2A, 0x01 }, 0xF3F4FB57u, 0xB4B5F0F7u }, // 帧 56000
    { 0x1B86u, 0x0CC8u, { 0x40, 0x01, 0xC3, 0x11, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x9E, 0x00 }, 0xF3F4FA57u, 0xB4B5F0F7u }, // 帧 56006
    { 0x1C48u, 0x0E01u, { 0x06, 0x05, 0xE5, 0x10, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x8A, 0x00 }, 0xF3F4FA57u, 0xB4B5F0F6u }, // 帧 56010
    { 0x1D86u, 0x0B06u, { 0xE2, 0x05, 0xF8, 0x0C, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xFB, 0x01 }, 0xF3F4FA57u, 0xB4B5EFF6u }, // 帧 56012
    { 0x1E07u, 0x0A1Bu, { 0xF4, 0x04, 0x69, 0x0C, 0xF4, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x6A, 0x01 }, 0xF3F4FA57u, 0xB4B5EFF7u }, // 帧 56014
    { 0x1E28u, 0x0A08u, { 0xFD, 0x02, 0x84, 0x0C, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x19, 0x01 }, 0xF3F4FA57u, 0xB4B5EFF6u }, // 帧 56016
    { 0x1D08u, 0x08B3u, { 0x6D, 0x02, 0xDD, 0x0A, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xAE, 0x00 }, 0xF3F4FA57u, 0xB4B5EFF7u }, // 帧 56022
    { 0x1CE6u, 0x0AC4u, { 0x8D, 0x03, 0x87, 0x0C, 0xF4, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x19, 0x01 }, 0xF3F4FA57u, 0xB4B5EEF7u }, // 帧 56033
    { 0x1C64u, 0x0A42u, { 0x1C, 0x03, 0x2A, 0x15, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0xE6, 0x00 }, 0xF3F4FA57u, 0xB4B5EEF7u }, // 帧 56035
    { 0x1CE0u, 0x0AE0u, { 0xFF, 0x03, 0x24, 0x0D, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x3D, 0x01 }, 0xF3F4FA57u, 0xB4B5EDF6u }, // 帧 56045
    { 0x1CEEu, 0x0AADu, { 0xC2, 0x02, 0xDB, 0x0C, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xFD, 0x00 }, 0xF3F4FA57u, 0xB4B5EDF7u }, // 帧 56047
    { 0x1D00u, 0x0A5Du, { 0x10, 0x02, 0xC5, 0x0C, 0xF4, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xD3, 0x00 }, 0xF3F4FA57u, 0xB4B5EDF7u }, // 帧 56049
    { 0x1CF7u, 0x09D4u, { 0xDE, 0x01, 0x2A, 0x0D, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xBB, 0x00 }, 0xF3F4FA57u, 0xB4B5EDF7u }, // 帧 56051
    { 0x1BCDu, 0x0C34u, { 0x03, 0x02, 0x35, 0x11, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x8D, 0x00 }, 0xF3F4FA57u, 0xB4B5EDF6u }, // 帧 56057
    { 0x1B9Du, 0x0EFDu, { 0x08, 0x02, 0xEB, 0x14, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0xCD, 0x00 }, 0xF3F4FA57u, 0xB4B5EDF7u }, // 帧 56064
    { 0x1E0Eu, 0x0AB0u, { 0x09, 0x05, 0xD5, 0x0C, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0xCD, 0x01 }, 0xF3F4FA57u, 0xB4B5EDF6u }, // 帧 56070
    { 0x1AF0u, 0x0E87u, { 0x3D, 0x01, 0xE9, 0x13, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x8C, 0x00 }, 0xF3F4FA57u, 0xB4B5EDF7u }, // 帧 56082
    { 0x1E26u, 0x0B3Du, { 0x8B, 0x01, 0xF1, 0x0C, 0xF3, 0xFF, 0xF2, 0xFF, 0x00, 0x00, 0x9E, 0x00 }, 0xF3F4FB57u, 0xB4B5EDF7u }, // 帧 56101
    { 0x1E42u, 0x0AFDu, { 0x45, 0x01, 0xCA, 0x0C, 0xF4, 0xFF, 0xF3, 0xFF, 0x00, 0x00, 0x8C, 0x00 }, 0xF3F4FB57u, 0xB4B5EDF7u }, // 帧 56103
};

/** @brief 0x7F 嗅探回放是否激活 */
static uint8_t before_on_use_sniff_7f_active = 0u;
/** @brief 当前在嗅探表中的索引位置 */
static uint8_t before_on_use_sniff_7f_index  = 0u;
/** @brief 0x7F 嗅探回放适用的通道号 */
static uint8_t before_on_use_sniff_7f_channel = 0xFFu;

/**
 * @brief 处理打印机发来的短帧运动命令（命令号 0x03）
 * 
 * 解析打印机的运动命令，调用 set_motion() 更新耗材状态，
 * 然后构建并发送 AMS 短帧运动状态回复包。
 * 
 * 回复包包含：
 * - 当前耗材使用标志和通道号
 * - 计米值（耗材消耗长度）
 * - 压力传感器值
 * - 各通道在线/运动状态标志
 * 
 * 对于 0x7F 准备状态，还会回放嗅探数据表中的特定回复序列。
 * 
 * @param package_recv 指向接收到的打印机运动命令包
 */
void get_package_motion(bambubus_printer_motion_package_struct *package_recv)
{
    if (bus_port_to_host.send_data_len != 0) return; // 上一个包尚未发送完毕
    uint8_t *out = bus_port_to_host.tx_build_buf();

    bambubus_printer_motion_package_struct in;
    memcpy(&in, package_recv, sizeof(in));

    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    if (in.ams_num != fixed_ams_num) return; // AMS 编号不匹配

    const uint8_t ams_idx = bambubus_ams_map[fixed_ams_num];
    if (!ams[ams_idx].online) return; // AMS 未在线

    _ams *ams_ptr = &ams[ams_idx];
    if (!set_motion(in.filamnet_channel, in.statu_flag, in.motion_flag, fixed_ams_num)) return;

    // 使用默认数据初始化回复包
    auto *package_send = (bambubus_ams_motion_package_struct *)out;
    memcpy(package_send, &_bambubus_ams_motion_package_struct_init_data, sizeof(*package_send));

    const uint8_t ch = ams_ptr->now_filament_num;
    const bool is_idle = (ch == 0xFF);               // 是否空闲（无通道在使用）
    const uint16_t pressure = is_idle ? 0xFF74 : ams_ptr->pressure; // 空闲时使用固定压力值

    package_send->flag = 0xC0 | (uint8_t)(package_num << 3); // 包含序列号的帧标志
    package_send->ams_num = fixed_ams_num;
    package_send->filament_use_flag = is_idle ? 0x00 : ams_ptr->filament_use_flag;
    package_send->filament_channel = ch;
    package_send->filament_channel_2 = ch;

    // 如果有通道在使用，复制计米值
    if (ch < 4u)
        memcpy(&package_send->meters, &ams_ptr->filament[ch].meters, sizeof(package_send->meters));

    memcpy(&package_send->pressure, &pressure, sizeof(pressure));

    // 空闲状态下设置特殊 unknow2 值
    if (is_idle)
        package_send->unknow2 = 0x0008;

    // 填充各通道在线/运动状态标志
    package_send->filament_stu_flag = get_filament_left_char(ams_ptr);

    // 判断是否需要回放 0x7F 准备状态的嗅探数据
    const bool replay_before_on_use_7f =
        (ch < 4u) &&
        (ams_ptr->filament[ch].motion == _filament_motion::before_on_use) &&
        (ams_ptr->filament_use_flag == 0x04u) &&
        (last_before_on_use_motion_flag == 0x7Fu);

    // 如果不需要回放，重置嗅探状态
    if (!replay_before_on_use_7f)
    {
        before_on_use_sniff_7f_active = 0u;
        before_on_use_sniff_7f_index = 0u;
        before_on_use_sniff_7f_channel = 0xFFu;
    }

    if (replay_before_on_use_7f)
    {
        // 初始化或继续嗅探回放
        if (!before_on_use_sniff_7f_active || before_on_use_sniff_7f_channel != ch)
        {
            before_on_use_sniff_7f_active = 1u;
            before_on_use_sniff_7f_index = 0u;
            before_on_use_sniff_7f_channel = ch;
        }

        const uint8_t rows_count = (uint8_t)(sizeof(before_on_use_sniff_7f_rows) / sizeof(before_on_use_sniff_7f_rows[0]));
        const uint8_t idx = before_on_use_sniff_7f_index;

        if (idx < rows_count)
        {
            // 从嗅探表中取数据填充回复包
            const before_on_use_sniff_row &row = before_on_use_sniff_7f_rows[idx];

            package_send->pressure = row.pressure;
            package_send->unknow2 = row.unknow2;
            memcpy(package_send->unknow3, row.unknow3, sizeof(row.unknow3));

            memcpy(out + 29, &row.last1, sizeof(row.last1));
            memcpy(out + 33, &row.last2, sizeof(row.last2));

            before_on_use_sniff_7f_index = (uint8_t)(idx + 1u); // 前进到下一帧
        }
        else
        {
            // 嗅探表已用完，使用默认值
            package_send->pressure = 0x1E34u;
            package_send->unknow2 = 0x0AFDu;
        }
    }
    else if (pressure == 0xF06Fu)
    {
        // 特殊压力值处理
        package_send->pressure = 0xF06Fu;
        package_send->unknow2 = 0x1CE7u;
    }

    // 递增包序号（循环 0~7）
    package_num = (package_num < 7u) ? (uint8_t)(package_num + 1u) : 0u;

    package_add_crc(out, sizeof(bambubus_ams_motion_package_struct)); // 添加 CRC 校验
    bus_port_to_host.send_data_len = sizeof(bambubus_ams_motion_package_struct); // 触发发送
}

/**
 * @brief 打印机发来的长帧运动命令包结构体（命令号 0x04）
 * 
 * 与短帧运动命令类似，但包含额外的温湿度数据字段。
 * 用于打印机请求 AMS 报告完整的环境状态信息。
 */
struct bambubus_printer_stu_motion_package_struct
{
    uint8_t magic_byte;         /**< 魔数 0x3D */
    uint8_t flag;              /**< 帧标志 0xC5（短帧） */
    uint8_t length;            /**< 数据包长度 */
    uint8_t crc8;             /**< 帧头 CRC8 校验 */
    uint8_t command;          /**< 命令号 0x04 */
    uint8_t ams_num;          /**< AMS 编号 */
    uint8_t statu_flag;       /**< 状态标志 */
    uint8_t motion_flag;      /**< 运动标志 */
    uint8_t unknow1;          /**< 未知字段1 */
    uint8_t filamnet_channel; /**< 耗材通道号 */
    uint8_t unknow2;          /**< 未知字段2 */
    uint16_t crc16;           /**< CRC16 校验 */
} __attribute__((packed));
static_assert(sizeof(bambubus_printer_stu_motion_package_struct) == 13, "packed size mismatch");

/**
 * @brief AMS 回复的长帧运动状态包结构体（命令号 0x04）
 * 
 * 比短帧回复包多出温湿度字段和在线标志字段。
 * 用于向打印机报告完整的 AMS 环境状态。
 */
struct bambubus_ams_stu_motion_package_struct
{
    uint8_t magic_byte = 0x3D;                  /**< 魔数 0x3D */
    uint8_t flag = 0xC0;                       /**< 帧标志 0xC0（短帧回复） */
    uint8_t length = 0x3C;                     /**< 数据包长度（60字节） */
    uint8_t crc8;                              /**< 帧头 CRC8 校验 */
    uint8_t command = 0x04;                    /**< 命令号 0x04 */
    uint8_t ams_num_stu = 0;                   /**< AMS 编号（状态包） */
    uint16_t temperature = 0;                  /**< 温度值，单位 0.1°C（4 个通道平均值） */
    uint8_t humidity = 0;                      /**< 湿度值，单位 %（4 个通道平均值） */
    uint8_t filament_online_flag[3];           /**< 耗材在线标志（3 字节冗余副本） */
    uint8_t filament_channel_stu = 0x00;       /**< 耗材通道状态 */
    uint8_t filament_flag_wait_NFC = 0x00;     /**< 等待 NFC 读取的标志 */
    uint8_t unknow_stu[3];                     /**< 未知字段（3字节） */
    uint8_t ams_num = 0;                       /**< AMS 编号（重复字段） */
    uint8_t unknow1 = 0x00;                   /**< 未知字段1 */
    uint8_t filament_use_flag = 0x00;          /**< 耗材使用标志 */
    uint8_t filament_channel = 0x00;           /**< 当前耗材通道号 */
    float meters = 0;                          /**< 计米值（米） */
    uint16_t pressure = 0;                     /**< 压力传感器值 */
    uint16_t unknow2 = 0xFFFF;                /**< 未知字段2 */
    uint8_t unknow3[12];                       /**< 未知字段3（12字节） */
    uint8_t filament_stu_flag = 0x00;          /**< 各通道在线/运动状态标志 */
    uint32_t last1 = 0xFFFFFFFF;              /**< 末尾填充字段1 */
    uint32_t last2 = 0x01010101;              /**< 末尾填充字段2 */
    uint32_t last3 = 0x00000000;              /**< 末尾填充字段3 */
    uint32_t last4 = 0xFFFFFFFF;              /**< 末尾填充字段4 */
    uint16_t crc16;                           /**< CRC16 校验 */
} __attribute__((packed));

/**
 * @brief AMS 长帧运动状态包的默认初始化数据
 * 
 * 作为模板使用，在构建回复包时先用此数据填充，再根据实际情况修改字段。
 */
static const bambubus_ams_stu_motion_package_struct _bambubus_ams_stu_motion_package_struct_init_data = {
    0x3D,       // magic_byte - 魔数标识
    0xC0,       // flag - 短帧回复标志
    0x3C,       // length - 包长度（60字节）
    0x00,       // crc8 - CRC8 校验
    0x04,       // command - 命令号
    0x00,       // ams_num_stu - AMS 编号
    0x0000,     // temperature - 温度
    0x00,       // humidity - 湿度
    {0x00,0x00,0x00}, // filament_online_flag[3] - 在线标志
    0x00,       // filament_channel_stu
    0x00,       // filament_flag_wait_NFC
    {0x00,0x00,0x00}, // unknow_stu[3]
    0x00,       // ams_num
    0x00,       // unknow1
    0x00,       // filament_use_flag
    0x00,       // filament_channel
    0.0f,       // meters
    0x0000,     // pressure
    0xFFFF,     // unknow2
    {           // unknow3[12]
        0x36, 0x00, 0x00, 0x00, 0xF9, 0xFF, 0xF8, 0xFF, 0x00, 0x00, 0x27, 0x00
    },
    0x00,           // filament_stu_flag
    0xF6F2F4FB,    // last1
    0xB1B4B7B5,    // last2
    0x00000000,    // last3
    0xFFFFFFFF,    // last4
    0x0000          // crc16
};

/**
 * @brief 处理打印机发来的长帧运动状态命令（命令号 0x04）
 * 
 * 解析打印机的运动命令，计算 4 个耗材通道的平均温度和湿度，
 * 调用 set_motion() 更新耗材状态，然后构建并发送 AMS 长帧运动状态回复包。
 * 
 * 温度计算：4 个通道温度之和 × 10 / 4（转换为 0.1°C 单位的平均值）
 * 湿度计算：4 个通道湿度之和 / 4（取平均值）
 * 
 * 回复包还包含：
 * - 温度和湿度数据
 * - 各通道在线标志
 * - 当前耗材使用状态和压力值
 * 
 * @param package_recv 指向接收到的打印机长帧运动命令包
 */
void get_package_stu_motion(bambubus_printer_stu_motion_package_struct *package_recv)
{
    if (bus_port_to_host.send_data_len != 0) return;
    uint8_t *out = bus_port_to_host.tx_build_buf();

    bambubus_printer_stu_motion_package_struct in;
    memcpy(&in, package_recv, sizeof(in));

    unsigned char filament_flag_on  = 0x00;  // 在线耗材标志
    unsigned char filament_flag_NFC = 0x00;  // NFC 读取标志

    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    if (in.ams_num != fixed_ams_num) return;

    const uint8_t ams_idx = bambubus_ams_map[fixed_ams_num];
    if (!ams[ams_idx].online) return;

    _ams *ams_ptr = &ams[ams_idx];

    // 统计各通道的在线状态
    for (uint8_t i = 0; i < 4u; i++)
        if (ams_ptr->filament[i].online)
            filament_flag_on |= (uint8_t)(1u << i); // 每个在线通道对应一个位

    if (!set_motion(in.filamnet_channel, in.statu_flag, in.motion_flag, fixed_ams_num)) return;

    // 使用默认数据初始化回复包
    auto *package_send = (bambubus_ams_stu_motion_package_struct *)out;
    memcpy(package_send, &_bambubus_ams_stu_motion_package_struct_init_data, sizeof(*package_send));

    // 计算 4 个通道的平均温度（0.1°C 单位）
    int16_t temperature = (int16_t)(
        ams_ptr->filament[0].compartment_temperature +
        ams_ptr->filament[1].compartment_temperature +
        ams_ptr->filament[2].compartment_temperature +
        ams_ptr->filament[3].compartment_temperature
    );
    temperature = (int16_t)(temperature * 10);   // 转换为 0.1°C 单位
    if (temperature < 0) temperature = 0;         // 温度不能为负
    temperature = (int16_t)(temperature >> 2);    // 除以 4 取平均值（右移 2 位）

    // 计算 4 个通道的平均湿度（% 单位）
    uint16_t humidity = (uint16_t)(
        ams_ptr->filament[0].compartment_humidity +
        ams_ptr->filament[1].compartment_humidity +
        ams_ptr->filament[2].compartment_humidity +
        ams_ptr->filament[3].compartment_humidity
    );
    humidity = (uint16_t)(humidity >> 2);         // 除以 4 取平均值

    const uint8_t ch = ams_ptr->now_filament_num;
    const bool is_idle = (ch == 0xFF);
    const uint16_t pressure = is_idle ? 0xFF74 : ams_ptr->pressure;

    package_send->flag = 0xC0 | (uint8_t)(package_num << 3);
    package_send->ams_num_stu = fixed_ams_num;
    package_send->temperature = (uint16_t)temperature;
    package_send->humidity = (uint8_t)humidity;

    // 计算在线标志（在线通道数 - NFC 通道数）
    const uint8_t on = (uint8_t)(filament_flag_on - filament_flag_NFC);
    package_send->filament_online_flag[0] = on;
    package_send->filament_online_flag[1] = on;
    package_send->filament_online_flag[2] = on;

    package_send->filament_channel_stu = in.filamnet_channel;
    package_send->filament_flag_wait_NFC = filament_flag_NFC;

    package_send->ams_num = fixed_ams_num;
    package_send->filament_use_flag = is_idle ? 0x00 : ams_ptr->filament_use_flag;
    package_send->filament_channel = ch;

    if (ch < 4)
        memcpy(&package_send->meters, &ams_ptr->filament[ch].meters, sizeof(package_send->meters));

    memcpy(&package_send->pressure, &pressure, sizeof(pressure));

    package_send->filament_stu_flag = get_filament_left_char(ams_ptr);

    // 特殊压力值处理
    if (pressure == 0xF06Fu)
    {
        package_send->pressure = 0xF06Fu;
        package_send->unknow2 = 0x1CE7u;
    }

    package_add_crc(out, sizeof(bambubus_ams_stu_motion_package_struct));

    package_num = (package_num < 7u) ? (uint8_t)(package_num + 1u) : 0u;

    bus_port_to_host.send_data_len = sizeof(bambubus_ams_stu_motion_package_struct);
}

/**
 * @brief 在线检测回复数据包（29 字节）
 * 
 * 打印机发送在线检测命令时，AMS 回复此数据包以确认自身在线。
 * 包含设备类型、序列号等信息，用于打印机注册 AMS。
 */
uint8_t online_detect_res[29] = {
    0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
    0x0D, 0x0E, 0xA0, '5', '5', '0', '0', 0x00, 0x00, '0', '0', '0', '0', 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
    0x33, 0xF0
};

extern unsigned char long_packge_version_serial_number[];

/**
 * @brief 构建在线检测回复数据包
 * 
 * 设置在线检测包的各字段：
 * - 魔数 0x3D、标志 0xC0、长度 29
 * - 命令号 0x05、子类型、AMS 编号
 * - 前缀值（0x0C 或 0x0A）
 * - 从序列号中复制 16 字节设备标识
 * 
 * @param ams_num AMS 编号
 * @param subtype 子类型（0x00=检测, 0x01=确认）
 */
static inline void online_detect_build_packet(const uint8_t ams_num, const uint8_t subtype)
{
    online_detect_res[0] = 0x3D;     // 魔数
    online_detect_res[1] = 0xC0;     // 标志
    online_detect_res[2] = 29;       // 长度
    online_detect_res[3] = 0xB4;     // CRC8（运行时计算）
    online_detect_res[4] = 0x05;     // 命令号
    online_detect_res[5] = subtype;  // 子类型
    online_detect_res[6] = ams_num;  // AMS 编号
    online_detect_res[7] = online_detect_prefix_now; // 前缀值
    memcpy(online_detect_res + 8, long_packge_version_serial_number + 33, 16); // 设备标识
    package_add_crc(online_detect_res, 29); // 添加 CRC16 校验
}

/**
 * @brief 处理打印机发来的在线检测命令（命令号 0x05）
 * 
 * 实现 AMS 在线检测和注册的完整流程：
 * 1. 子类型 0x00：打印机检测 AMS 是否在线，AMS 回复包含前缀值的检测包
 * 2. 子类型 0x01：打印机确认注册，AMS 验证前缀值匹配后标记为已注册
 * 
 * 在线检测使用两阶段前缀值（0x0C 和 0x0A）进行握手确认。
 * 
 * @param buf 接收到的数据包
 * @param length 数据包长度
 */
void get_package_online_detect(unsigned char *buf, int length)
{
    (void)length;
    if (bus_port_to_host.send_data_len != 0) return;

    const uint8_t ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    if (ams_num >= 4u) return;

    // AMS 未在线，重置检测状态
    if (ams[bambubus_ams_map[ams_num]].online != true)
    {
        online_detect_reset();
        return;
    }

    if (buf[5] == 0x00) // 子类型 0x00：打印机检测阶段
    {
        if (have_registered) return; // 已注册则不再回复

        if (online_detect_phase == 0u)
        {
            online_detect_prefix_now = 0x0Cu; // 第一次检测使用前缀 0x0C
            online_detect_phase = 1u;
        }
        else
        {
            online_detect_prefix_now = 0x0Au; // 重复检测使用前缀 0x0A
            online_detect_phase = 2u;
        }

        online_detect_build_packet(ams_num, 0x00);

        uint8_t *out = bus_port_to_host.tx_build_buf();
        memcpy(out, online_detect_res, 29);
        bus_port_to_host.send_data_len = 29;
        return;
    }

    if (buf[5] != 0x01) return;   // 只处理子类型 0x01
    if (buf[6] != ams_num) return; // AMS 编号必须匹配

    online_detect_prefix_now = 0x0Au;
    online_detect_build_packet(ams_num, 0x01);

    // 验证打印机回复中的前缀和设备标识是否匹配
    if (memcmp(online_detect_res + 7, buf + 7, 17) != 0)
        return;

    have_registered = true;     // 标记为已注册
    online_detect_phase = 3u;

    uint8_t *out = bus_port_to_host.tx_build_buf();
    memcpy(out, online_detect_res, 29);
    bus_port_to_host.send_data_len = 29;
}

/**
 * @brief 处理打印机发来的主控在线确认命令（长帧类型 0x21A）
 * 
 * 打印机通过长帧发送主控在线确认，AMS 回复一个包含 AMS 编号的确认包。
 * 用于建立长帧通信连接。
 * 
 * @param buf 接收到的数据包
 * @param length 数据包长度
 */
void get_package_long_packge_MC_online(unsigned char *buf, int length)
{
    (void)buf;
    (void)length;

    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;

    if (printer_data_long.data_length < 1u) return;
    if (!ams[bambubus_ams_map[fixed_ams_num]].online) return;
    if (printer_data_long.datas[0] != fixed_ams_num) return; // 验证 AMS 编号

    // 构建回复数据：6 字节确认包
    unsigned char resp[6] = {fixed_ams_num, 0x00, 0x00, 0x00, 0x00, 0x00};

    // 设置长帧回复参数
    bambubus_long_packge_data data;
    data.datas = resp;
    data.data_length = sizeof(resp);
    data.package_number = printer_data_long.package_number;  // 回复相同的序列号
    data.type = printer_data_long.type;                       // 回复相同的命令类型
    data.source_address = printer_data_long.target_address;  // 源地址 = 原目标地址
    data.target_address = printer_data_long.source_address;  // 目标地址 = 原源地址

    bambubus_long_package_get(&data); // 构建并发送长帧
}

/**
 * @brief 耗材信息回复数据包（110 字节）
 * 
 * 包含单个耗材通道的完整信息：
 * - 通道号和耗材 ID
 * - 耗材名称
 * - RGBA 颜色值
 * - 温度范围
 * - 其他元数据
 */
unsigned char long_packge_filament[] =
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // [0-18] 通道号和填充
        0x47, 0x46, 0x42, 0x30, 0x30, 0x00, 0x00, 0x00, // [19-26] 默认耗材 ID "GFB00"
        0x41, 0x42, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // [27-46] 默认耗材名称 "ABS"
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // [47-58] 填充
        0xDD, 0xB1, 0xD4, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // [59-78] RGBA 颜色值
        0x18, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // [79-98] 温度范围
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // [99-109] 填充
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // [110+] 填充

/**
 * @brief 处理打印机发来的读取耗材信息命令（长帧类型 0x211）
 * 
 * 打印机通过长帧请求读取指定 AMS 中指定通道的耗材信息。
 * AMS 回复包含耗材 ID、名称、颜色、温度范围等完整信息。
 * 
 * @param buf 接收到的数据包
 * @param length 数据包长度
 */
void get_package_long_packge_filament(unsigned char *buf, int length)
{
    (void)buf;
    (void)length;

    bambubus_long_packge_data data;

    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    const uint8_t ams_num = printer_data_long.datas[0];     // 目标 AMS 编号
    const uint8_t filament_num = printer_data_long.datas[1]; // 目标通道号

    // 验证 AMS 编号和通道号
    if (ams_num != fixed_ams_num || filament_num >= 4 || ams[bambubus_ams_map[fixed_ams_num]].online != true)
    {
        return;
    }

    _ams *ams_ptr = ams + bambubus_ams_map[fixed_ams_num];
    long_packge_filament[0] = fixed_ams_num;    // 填充 AMS 编号
    long_packge_filament[1] = filament_num;     // 填充通道号
    // 复制耗材 ID（8 字节）
    memcpy(long_packge_filament + 19, ams_ptr->filament[filament_num].bambubus_filament_id, sizeof(ams_ptr->filament[filament_num].bambubus_filament_id));
    // 复制耗材名称（20 字节）
    memcpy(long_packge_filament + 27, ams_ptr->filament[filament_num].name, sizeof(ams_ptr->filament[filament_num].name));
    // 复制 RGBA 颜色值
    long_packge_filament[59] = ams_ptr->filament[filament_num].color_R;
    long_packge_filament[60] = ams_ptr->filament[filament_num].color_G;
    long_packge_filament[61] = ams_ptr->filament[filament_num].color_B;
    long_packge_filament[62] = ams_ptr->filament[filament_num].color_A;
    // 复制温度范围
    memcpy(long_packge_filament + 79, &ams_ptr->filament[filament_num].temperature_max, 2);
    memcpy(long_packge_filament + 81, &ams_ptr->filament[filament_num].temperature_min, 2);

    // 设置长帧回复参数
    data.datas = long_packge_filament;
    data.data_length = sizeof(long_packge_filament);
    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;

    bambubus_long_package_get(&data);
}

/**
 * @brief 版本和序列号回复数据包
 * 
 * 包含设备的版本号、序列号等信息。
 * 序列号基于芯片唯一 ID 通过 FNV-1a 哈希算法生成，确保每个设备唯一。
 * 
 * 布局：
 * - [0]: 版本号长度（固定为 15）
 * - [1-15]: 版本号字符串（如 "0EA0XXXXXXXXXX"）
 * - [16-32]: 填充
 * - [33-48]: 序列号二进制数据
 * - [49-64]: 填充
 * - [65]: AMS 编号
 * - [66-77]: 其他元数据
 */
unsigned char long_packge_version_serial_number[] = {15,
                                                     '0', 'E', 'A', '0', '3', '0', '3', '0', '3', '0', '0', '0', '0', '0', '0', // 版本号
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                     0x00, // 填充
                                                     0x0E, 0xA0, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, // 序列号前缀
                                                     0x30, 0x30, 0x30, 0x30, // 序列号
                                                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 填充
                                                     0x00, // 填充
                                                     0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, // 元数据
                                                     0x00}; // 结尾

/**
 * @brief 构建基于芯片唯一 ID 的静态序列号
 * 
 * 使用 FNV-1a 哈希算法（64位版本）对芯片 12 字节唯一 ID 进行哈希，
 * 生成唯一的序列号。序列号同时以十六进制字符串和二进制形式存储。
 * 
 * 哈希算法参数：
 * - 偏移_basis: 1469598103934665603 (0xcbf29ce484222325)
 * - 乘法因子: 1099511628211 (0x100000001b3)
 * 
 * 还使用了 MurmurHash3 的混合函数对哈希值进行最终处理。
 */
static void bambubus_build_static_serial(void)
{
    static const char hex[] = "0123456789ABCDEF";
    volatile const uint8_t *uid = (volatile const uint8_t *)0x1FFFF7E8; // CH32V203 芯片唯一 ID 地址
    const uint8_t ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;

    // FNV-1a 哈希初始化
    uint64_t v = 1469598103934665603ull;
    for (int i = 0; i < 12; i++)
    {
        v ^= uid[i];
        v *= 1099511628211ull;
    }

    // MurmurHash3 混合函数
    v ^= v >> 30;
    v *= 0xBF58476D1CE4E5B9ull;
    v ^= v >> 27;
    v *= 0x94D049BB133111EBull;
    v ^= v >> 31;

    // 构建版本号字符串
    long_packge_version_serial_number[0] = 15;        // 版本号长度
    long_packge_version_serial_number[1] = '0';       // 固定前缀
    long_packge_version_serial_number[2] = 'E';
    long_packge_version_serial_number[3] = 'A';
    long_packge_version_serial_number[4] = '0' + ams_num; // AMS 编号

    {
        // 将哈希值的前 6 字节转换为十六进制字符串
        const uint8_t b0 = (uint8_t)(v >> 56);
        const uint8_t b1 = (uint8_t)(v >> 48);
        const uint8_t b2 = (uint8_t)(v >> 40);
        const uint8_t b3 = (uint8_t)(v >> 32);
        const uint8_t b4 = (uint8_t)(v >> 24);
        const uint8_t b5 = (uint8_t)(v >> 16);

        long_packge_version_serial_number[5]  = hex[(b0 >> 4) & 0x0F];
        long_packge_version_serial_number[6]  = hex[b0 & 0x0F];
        long_packge_version_serial_number[7]  = hex[(b1 >> 4) & 0x0F];
        long_packge_version_serial_number[8]  = hex[b1 & 0x0F];
        long_packge_version_serial_number[9]  = hex[(b2 >> 4) & 0x0F];
        long_packge_version_serial_number[10] = hex[b2 & 0x0F];
        long_packge_version_serial_number[11] = hex[(b3 >> 4) & 0x0F];
        long_packge_version_serial_number[12] = hex[b3 & 0x0F];
        long_packge_version_serial_number[13] = hex[(b4 >> 4) & 0x0F];
        long_packge_version_serial_number[14] = hex[b4 & 0x0F];
        long_packge_version_serial_number[15] = hex[(b5 >> 4) & 0x0F];
    }

    // 存储二进制格式的序列号
    long_packge_version_serial_number[33] = 0x0E;
    long_packge_version_serial_number[34] = (uint8_t)(0xA0 + ams_num);
    long_packge_version_serial_number[35] = (uint8_t)(v >> 56);
    long_packge_version_serial_number[36] = (uint8_t)(v >> 48);
    long_packge_version_serial_number[37] = (uint8_t)(v >> 40);
    long_packge_version_serial_number[38] = (uint8_t)(v >> 32);
    long_packge_version_serial_number[39] = (uint8_t)(v >> 24);
    long_packge_version_serial_number[40] = (uint8_t)(v >> 16);
    long_packge_version_serial_number[41] = (uint8_t)(v >> 24);
    long_packge_version_serial_number[42] = (uint8_t)(v >> 16);
    long_packge_version_serial_number[43] = (uint8_t)(v >> 8);
    long_packge_version_serial_number[44] = (uint8_t)(v >> 0);
    long_packge_version_serial_number[45] = 0xFF;
    long_packge_version_serial_number[46] = 0xFF;
    long_packge_version_serial_number[47] = 0xFF;
    long_packge_version_serial_number[48] = 0xFF;
    long_packge_version_serial_number[65] = ams_num;   // AMS 编号

    online_detect_reset();
    online_detect_res[6] = ams_num;                    // 更新在线检测包的 AMS 编号
    memcpy(online_detect_res + 8, long_packge_version_serial_number + 33, 16); // 复制设备标识
}

/**
 * @brief 处理打印机发来的序列号查询命令（长帧类型 0x402）
 * 
 * 打印机通过长帧请求读取 AMS 的序列号信息。
 * AMS 回复包含基于芯片 UID 生成的唯一序列号。
 * 
 * @param buf 接收到的数据包
 * @param length 数据包长度
 */
void get_package_long_packge_serial_number(unsigned char *buf, int length)
{
    (void)buf;
    (void)length;

    const uint8_t ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;

    // 验证请求中的 AMS 编号
    if ((printer_data_long.data_length > 33) && (printer_data_long.datas[33] != ams_num))
    {
        return;
    }

    if (ams[bambubus_ams_map[ams_num]].online != true)
    {
        return;
    }

    // 构建长帧回复
    bambubus_long_packge_data data;
    data.datas = long_packge_version_serial_number;
    data.data_length = sizeof(long_packge_version_serial_number);
    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    bambubus_long_package_get(&data);
}

/**
 * @brief 版本号和设备名称回复数据包（AMS08 型号）
 * 
 * 包含固件版本号（50 = 0x32）和设备型号名称 "AMS08"。
 * 版本号采用 BCD 编码：0x0A=10, 0x14=20, 0x1E=30, 0x28=40, 0x32=50
 */
unsigned char long_packge_version_version_and_name_AMS08[] = {0x00, 0x00, 0x32, 0x0A , // 版本号（50）
                                                              0x41, 0x4D, 0x53, 0x30, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 设备名称 "AMS08"

/**
 * @brief 处理打印机发来的版本查询命令（长帧类型 0x103）
 * 
 * 打印机通过长帧查询 AMS 的固件版本和设备型号。
 * AMS 回复包含版本号和设备名称的数据包。
 * 
 * @param buf 接收到的数据包
 * @param length 数据包长度
 */
void get_package_long_packge_version(unsigned char *buf, int length)
{
    (void)buf;
    (void)length;

    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    const uint8_t ams_num = printer_data_long.datas[0];

    if (ams_num != fixed_ams_num || ams[bambubus_ams_map[fixed_ams_num]].online != true)
        return;

    // 在版本数据末尾填入 AMS 编号
    long_packge_version_version_and_name_AMS08[sizeof(long_packge_version_version_and_name_AMS08) - 1u] = fixed_ams_num;

    // 构建长帧回复
    bambubus_long_packge_data data;
    data.datas = long_packge_version_version_and_name_AMS08;
    data.data_length = (uint16_t)sizeof(long_packge_version_version_and_name_AMS08);
    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;

    bambubus_long_package_get(&data);
}

/** @brief 设置耗材信息的短帧回复包（8 字节） */
unsigned char set_filament_res[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};

/**
 * @brief 处理打印机发来的设置耗材信息命令（短帧命令号 0x08）
 * 
 * 打印机通过短帧向 AMS 写入指定通道的耗材信息，包括：
 * - 耗材 ID（8 字节）
 * - RGBA 颜色值（4 字节）
 * - 温度范围（2+2 字节）
 * - 耗材名称（最多 20 字节）
 * 
 * 写入后触发保存操作，将数据持久化到 Flash。
 * 
 * @param buf 接收到的数据包
 * @param length 数据包长度
 */
void get_package_set_filament(unsigned char *buf, int length)
{
    (void)length;

    if (bus_port_to_host.send_data_len != 0) return;
    uint8_t* out = bus_port_to_host.tx_build_buf();
    uint8_t b = buf[5]; // AMS 编号和通道号编码在 buf[5] 中

    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    uint8_t ams_num  = (b >> 4) & 0x0F;   // 高 4 位为 AMS 编号
    uint8_t read_num = (b >> 0) & 0x0F;   // 低 4 位为通道号

    if (ams_num != fixed_ams_num || read_num >= 4 || ams[bambubus_ams_map[fixed_ams_num]].online != true) return;

    _ams *ams_ptr = ams + bambubus_ams_map[fixed_ams_num];
    // 复制耗材 ID（8 字节，从 buf[7] 开始）
    memcpy(ams_ptr->filament[read_num].bambubus_filament_id, buf + 7, sizeof(ams_ptr->filament[read_num].bambubus_filament_id));
    // 复制 RGBA 颜色值
    ams_ptr->filament[read_num].color_R = buf[15];
    ams_ptr->filament[read_num].color_G = buf[16];
    ams_ptr->filament[read_num].color_B = buf[17];
    ams_ptr->filament[read_num].color_A = buf[18];
    // 复制温度范围
    memcpy(&ams_ptr->filament[read_num].temperature_min, buf + 19, 2);
    memcpy(&ams_ptr->filament[read_num].temperature_max, buf + 21, 2);
    // 复制耗材名称（20 字节，从 buf[23] 开始）
    memcpy(ams_ptr->filament[read_num].name, buf + 23, sizeof(ams_ptr->filament[read_num].name));
    ams_ptr->filament[read_num].name[19] = 0; // 确保字符串以 null 结尾

    // 发送固定回复包
    memcpy(out, set_filament_res, sizeof(set_filament_res));
    bus_port_to_host.send_data_len = sizeof(set_filament_res);
}

/** @brief 设置耗材信息类型2 的长帧回复包（3 字节） */
unsigned char set_filament_res_type2[] = {0x00, 0x00, 0x00};

/**
 * @brief 处理打印机发来的设置耗材信息类型2 命令（长帧类型 0x218）
 * 
 * 与短帧版本功能相同，但通过长帧传输。
 * 数据布局不同：
 * - datas[0]: AMS 编号
 * - datas[1]: 通道号
 * - datas[2-9]: 耗材 ID
 * - datas[10-13]: RGBA 颜色值
 * - datas[14-15]: 最低温度
 * - datas[16-17]: 最高温度
 * - datas[18-33]: 耗材名称
 * 
 * @param buf 接收到的数据包
 * @param length 数据包长度
 */
void get_package_set_filament_type2(unsigned char *buf, int length)
{
    if (bus_port_to_host.send_data_len != 0) return;
    (void)buf;
    (void)length;

    bambubus_long_packge_data data;

    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    const uint8_t ams_num  = printer_data_long.datas[0]; // AMS 编号
    const uint8_t read_num = printer_data_long.datas[1]; // 通道号

    if (ams_num != fixed_ams_num || read_num >= 4 || ams[bambubus_ams_map[fixed_ams_num]].online != true) return;

    _ams *ams_ptr = ams + bambubus_ams_map[fixed_ams_num];

    // 复制耗材 ID（8 字节，从 datas[2] 开始）
    memcpy(ams_ptr->filament[read_num].bambubus_filament_id,
           printer_data_long.datas + 2,
           sizeof(ams_ptr->filament[read_num].bambubus_filament_id));

    // 复制 RGBA 颜色值
    ams_ptr->filament[read_num].color_R = printer_data_long.datas[10];
    ams_ptr->filament[read_num].color_G = printer_data_long.datas[11];
    ams_ptr->filament[read_num].color_B = printer_data_long.datas[12];
    ams_ptr->filament[read_num].color_A = printer_data_long.datas[13];

    // 复制温度范围
    memcpy(&ams_ptr->filament[read_num].temperature_min, printer_data_long.datas + 14, 2);
    memcpy(&ams_ptr->filament[read_num].temperature_max, printer_data_long.datas + 16, 2);
    // 复制耗材名称（16 字节，从 datas[18] 开始）
    memset(ams_ptr->filament[read_num].name, 0, sizeof(ams_ptr->filament[read_num].name));
    memcpy(ams_ptr->filament[read_num].name, printer_data_long.datas + 18, 16);
    ams_ptr->filament[read_num].name[19] = 0;

    // 构建长帧回复
    set_filament_res_type2[0] = fixed_ams_num;
    set_filament_res_type2[1] = read_num;
    set_filament_res_type2[2] = 0x00; // 成功标志

    data.datas = set_filament_res_type2;
    data.data_length = sizeof(set_filament_res_type2);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;

    bambubus_long_package_get(&data);
}

/**
 * @brief BambuBus 协议主运行函数
 * 
 * 在主循环中周期性调用，处理 BambuBus 总线上的所有通信。
 * 
 * 处理流程：
 * 1. 从总线端口读取接收到的数据包（临界区保护）
 * 2. 验证包的有效性（魔数 0x3D、CRC16 校验）
 * 3. 解析包类型并分发到对应的处理函数
 * 4. 处理函数构建回复包后，延迟 50μs 确保发送完成
 * 5. 清空接收缓冲区，准备接收下一个包
 * 6. 检测心跳超时，返回相应的状态码
 * 
 * @return bambubus_package_type 当前处理的状态
 *   - heartbeat: 收到心跳包或心跳截止时间到达
 *   - error: 心跳超时（超过 1000ms 未收到数据包）
 *   - 其他: 对应的数据包类型
 */
bambubus_package_type bambubus_run()
{
    bambubus_package_type stu = bambubus_package_type::none;

    static uint32_t last_hb_deadline = 0u;    // 上次处理的心跳截止时间
    const uint32_t now = time_ticks32();

    int rx_len = 0;
    _bus_data_type t = _bus_data_type::none;
    uint8_t *buf = nullptr;

    // 临界区：读取接收数据（避免与中断竞争）
    {
        const uint32_t s = irq_save_wch();
        rx_len = bus_port_to_host.recv_data_len;
        t = bus_port_to_host.bus_package_type;
        buf = bus_port_to_host.bus_recv_data_ptr;
        irq_restore_wch(s);
    }

    // 检查是否收到有效的 BambuBus 数据包
    if (rx_len > 0 && t == _bus_data_type::bambubus)
    {
        if (buf != nullptr && rx_len <= 1280 && buf[0] == 0x3D)
        {
            const int len = rx_len;

            stu = get_packge_type(buf, len); // 解析包类型

            // 根据包类型分发到对应的处理函数
            switch (stu)
            {
            case bambubus_package_type::filament_motion_short:
                get_package_motion((bambubus_printer_motion_package_struct *)buf);
                break;

            case bambubus_package_type::filament_motion_long:
                get_package_stu_motion((bambubus_printer_stu_motion_package_struct *)buf);
                break;

            case bambubus_package_type::online_detect:
                get_package_online_detect(buf, len);
                break;

            case bambubus_package_type::MC_online:
                get_package_long_packge_MC_online(buf, len);
                break;

            case bambubus_package_type::read_filament_info:
                get_package_long_packge_filament(buf, len);
                break;

            case bambubus_package_type::version:
                get_package_long_packge_version(buf, len);
                break;

            case bambubus_package_type::serial_number:
                get_package_long_packge_serial_number(buf, len);
                break;

            case bambubus_package_type::set_filament_info:
            {
                const uint8_t b = buf[5];
                const uint8_t ams_num = (b >> 4) & 0x0F;
                const uint8_t fil = (b >> 0) & 0x0F;

                get_package_set_filament(buf, len);

                // 如果是当前 AMS 且通道号有效，触发保存操作
                if (ams_num == (uint8_t)BAMBU_BUS_AMS_NUM && fil < 4)
                    ams_datas_set_need_to_save_filament(fil);
                break;
            }

            case bambubus_package_type::set_filament_info_type2:
                get_package_set_filament_type2(buf, len);
                // 如果是当前 AMS 且通道号有效，触发保存操作
                if (printer_data_long.datas[0] == (uint8_t)BAMBU_BUS_AMS_NUM && printer_data_long.datas[1] < 4)
                    ams_datas_set_need_to_save_filament(printer_data_long.datas[1]);
                break;

            default:
                break;
            }

            // 等待 50μs 确保回复包发送完成
            if (bus_port_to_host.send_data_len != 0) delay_us(50u);
        }

        // 临界区：清空接收缓冲区
        {
            const uint32_t s = irq_save_wch();
            bus_port_to_host.recv_data_len = 0;
            bus_port_to_host.bus_package_type = _bus_data_type::none;
            irq_restore_wch(s);
        }
    }

    // 心跳检测逻辑
    uint32_t hb_deadline = 0u;
    {
        const uint32_t s = irq_save_wch();
        hb_deadline = bambubus_heartbeat_deadline;
        irq_restore_wch(s);
    }

    // 如果没有收到数据包但心跳截止时间已更新，报告心跳状态
    if (stu == bambubus_package_type::none && hb_deadline != last_hb_deadline)
    {
        last_hb_deadline = hb_deadline;
        if (time_diff32(hb_deadline, now) > 0)
            stu = bambubus_package_type::heartbeat; // 心跳未超时
    }

    // 心跳超时检测：当前时间超过截止时间
    if (time_diff32(now, hb_deadline) > 0)
        stu = bambubus_package_type::error;

    return stu;
}
