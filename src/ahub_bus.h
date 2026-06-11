#pragma once
#include <stdint.h>

/**
 * @brief 启用 xMCU 模式标志
 * 
 * 定义此宏后，AHUB 协议中的地址格式采用 xMCU 编码方式：
 * 地址的高 4 位为 AMS 编号，低 2 位为通道编号。
 * 未定义时使用简单编号方式。
 */
#define xMCU

/**
 * @brief AHUB 协议数据包类型枚举
 * 
 * 定义了 AHUB（Another Hub）总线上主控与 AMS 之间通信的所有数据包类型。
 * AHUB 是一种简化的集线器协议，用于多 AMS 设备的管理和数据交换。
 */
enum class ahubus_package_type : uint8_t
{
    heartbeat = 0x01,   /**< 心跳包，用于检测通信链路是否存活 */
    query = 0x02,       /**< 查询命令，主控请求 AMS 返回状态信息 */
    set = 0x03,         /**< 设置命令，主控向 AMS 写入配置或状态 */
    none,               /**< 无有效数据包 */
    error,              /**< 通信错误（心跳超时等） */
};

/**
 * @brief AHUB 设置命令子类型枚举
 * 
 * 定义了 AHUB set 命令支持的所有子类型，用于区分不同的设置操作。
 */
enum class ahubus_set_type : uint8_t
{
    filament_info = 0x02,       /**< 设置耗材信息（ID、颜色、温度等） */
    dryer_stu = 0x05,           /**< 设置烘干机状态（功率、温度、时间） */
    all_filament_stu = 0x06,    /**< 设置所有耗材状态（运动状态、在线状态） */
};

/**
 * @brief 初始化 AHUB 协议栈
 * 
 * 启用 CRC 硬件外设时钟，为 AHUB 协议的 CRC 校验做准备。
 */
extern void ahubus_init();

/**
 * @brief AHUB 协议主运行函数（主循环调用）
 * 
 * 从总线端口读取接收到的 AHUB 数据包，解析包类型，
 * 执行相应的命令处理函数，并构建响应数据包。
 * 同时检测心跳超时状态。
 * 
 * @return ahubus_package_type 当前处理的数据包类型或通信状态
 */
extern ahubus_package_type ahubus_run();
