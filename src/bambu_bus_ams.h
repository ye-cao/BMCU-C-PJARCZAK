#pragma once
#include <stdint.h>

/**
 * @brief BambuBus 协议数据包类型枚举
 * 
 * 定义了 BambuBus 总线上打印机与 AMS 之间通信的所有数据包类型。
 * 每种类型对应一种特定的命令或响应，用于控制 AMS（自动供料系统）的各种功能。
 */
enum class bambubus_package_type
{
    error = -1,             /**< 通信错误，通常表示心跳超时或 CRC 校验失败 */
    none = 0,               /**< 无有效数据包，或数据包尚未到达 */
    filament_motion_short,  /**< 短帧耗材运动命令 (命令号 0x03)，打印机请求 AMS 报告耗材运动状态 */
    filament_motion_long,   /**< 长帧耗材状态命令 (命令号 0x04)，打印机请求 AMS 报告含温湿度的完整状态 */
    online_detect,          /**< 在线检测命令 (命令号 0x05)，打印机检测 AMS 是否在线并注册 */
    REQx6,                  /**< 请求命令 0x06，功能待确认 */
    NFC_detect,             /**< NFC 检测命令 (命令号 0x07)，打印机请求读取 NFC 标签信息 */
    set_filament_info,      /**< 设置耗材信息命令 (命令号 0x08)，打印机向 AMS 写入耗材 ID/颜色/温度等 */
    MC_online,              /**< 主控在线确认 (长帧类型 0x21A)，打印机确认主控通信连接 */
    read_filament_info,     /**< 读取耗材信息 (长帧类型 0x211)，打印机请求 AMS 返回耗材详细信息 */
    set_filament_info_type2,/**< 设置耗材信息类型2 (长帧类型 0x218)，打印机通过长帧写入耗材信息 */
    version,                /**< 版本查询 (长帧类型 0x103)，打印机请求 AMS 返回固件版本号 */
    serial_number,          /**< 序列号查询 (长帧类型 0x402)，打印机请求 AMS 返回设备序列号 */
    heartbeat,              /**< 心跳包，由定时器触发，用于检测通信链路是否存活 */
    ETC,                    /**< 其他未分类的数据包类型 */

    __BambuBus_package_packge_type_size  /**< 枚举大小标记，用于编译期检查 */
};

/**
 * @brief 初始化 BambuBus 协议栈
 * 
 * 调用静态序列号构建函数，生成基于芯片 UID 的唯一序列号，
 * 并重置心跳超时计数器。
 */
void bambubus_init(void);

/**
 * @brief 快速更新心跳截止时间（从 ISR 中调用）
 * 
 * 当收到有效的 BambuBus 数据包时，此函数会被调用以刷新心跳超时计时器。
 * 心跳超时设置为 1000 毫秒，超过此时间未收到数据包则判定通信断开。
 */
void bambubus_heartbeat_seen_fast(void);

/**
 * @brief BambuBus 协议主运行函数（主循环调用）
 * 
 * 从总线端口读取接收到的数据包，解析包类型，执行相应的命令处理函数，
 * 并构建响应数据包发送回打印机。同时检测心跳超时状态。
 * 
 * @return bambubus_package_type 当前处理的数据包类型，用于上层逻辑判断通信状态
 */
extern bambubus_package_type bambubus_run();
