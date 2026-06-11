#include "ahub_bus.h"

#include <string.h>

#include "ch32v20x_rcc.h"
#include "ch32v20x_crc.h"
#include "hal/irq_wch.h"
#include "hal/time_hw.h"
#include "app_api.h"
#include "_bus_hardware.h"
#include "ams.h"
#include "crc_bus.h"

/**
 * @brief 32位无符号整数的别名类型，启用 may_alias 属性
 * 
 * 用于绕过严格别名规则（strict aliasing），确保通过 uint8_t* 指针
 * 访问内存时不会被编译器优化掉。在 CRC 硬件计算中需要将 uint8_t*
 * 转换为 uint32_t* 进行访问。
 */
typedef uint32_t u32_alias __attribute__((may_alias));

/**
 * @brief 将端口号和地址编码为 AHUB 索引
 * 
 * @param port 端口号（高 4 位）
 * @param adr 地址（右移 2 位后作为索引）
 * @return 编码后的索引值
 */
#define ahubus_map_port_adr_to_index(port, adr) (((uint8_t)port << 4) + ((uint8_t)adr >> 2))

/**
 * @brief 获取 AHUB 数据包类型
 * 
 * 验证数据包的有效性并返回包类型：
 * 1. 检查魔数 0x33
 * 2. 使用 CH32V203 硬件 CRC 模块计算 CRC32 校验
 * 3. 比较计算结果与包中的 CRC 值
 * 4. 返回命令类型字段
 * 
 * @param package_recv_buf 接收到的数据包缓冲区
 * @return ahubus_package_type 解析出的数据包类型
 */
ahubus_package_type ahubus_get_package_type(uint8_t *package_recv_buf)
{
    if (package_recv_buf == nullptr) return ahubus_package_type::none;
    if (package_recv_buf[0] != 0x33) return ahubus_package_type::none; // 魔数校验

    const uint32_t words = (uint32_t)package_recv_buf[2] + 2u; // CRC 计算的 32 位字数
    const u32_alias *w = (const u32_alias *)package_recv_buf;

    // 使用硬件 CRC 模块计算 CRC32
    CRC->CTLR = 1; // 复位 CRC 寄存器
    for (uint32_t i = 0; i < words; i++)
        CRC->DATAR = w[i]; // 逐字送入 CRC 数据寄存器

    // 比较计算结果与包中的 CRC 值
    if ((uint32_t)CRC->DATAR != (uint32_t)w[words])
        return ahubus_package_type::none; // CRC 校验失败

    return (ahubus_package_type)package_recv_buf[4]; // 命令类型位于偏移 4
}

/**
 * @brief 初始化 AHUB 协议
 * 
 * 启用 CH32V203 的 CRC 硬件外设时钟，为后续的 CRC32 计算做准备。
 */
void ahubus_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}

/**
 * @brief 为 AHUB 数据包添加 CRC 校验
 * 
 * 先计算 CRC8 校验（对前 3 字节），再使用硬件 CRC 模块计算 CRC32 校验。
 * CRC32 结果追加到包尾。
 * 
 * @param buf 指向待添加校验的数据包缓冲区
 * @return int 数据包总长度（字节，含 CRC32 的 4 字节）
 */
int ahubus_package_add_crc(uint8_t *buf)
{
    if (buf == nullptr) return 0;

    const uint32_t words = (uint32_t)buf[2] + 2u; // CRC 计算的 32 位字数
    u32_alias *w = (u32_alias *)buf;

    buf[3] = bus_crc8(buf, 3); // CRC8 校验（对前 3 字节）

    // 使用硬件 CRC 模块计算 CRC32
    CRC->CTLR = 1; // 复位 CRC 寄存器
    for (uint32_t i = 0; i < words; i++)
        CRC->DATAR = w[i];

    w[words] = (uint32_t)CRC->DATAR; // 将 CRC32 追加到包尾
    return (int)((words + 1u) << 2); // 返回总长度（字节）
}

/**
 * @brief 处理主控发来的心跳包
 * 
 * 构建心跳回复包，包含当前在线的所有 AMS 设备信息：
 * - 设备地址和类型
 * - 在线 AMS 数量
 * 
 * 心跳包格式：
 * - 魔数 0x33
 * - 标志位
 * - 长度（32 位字数）
 * - CRC8
 * - 命令号 0x01（心跳）
 * - AMS 数量
 * - 各在线 AMS 的地址和类型（每 2 字节一组）
 * - 填充（确保偶数个 AMS 条目）
 * - CRC32
 * 
 * @param buf 接收到的心跳包数据
 */
void ahubus_slave_get_package_heartbeat(uint8_t *buf)
{
    if (bus_port_to_host.send_data_len != 0) return; // 上一个包尚未发送完毕
    uint8_t* out = bus_port_to_host.tx_build_buf();

    uint8_t ahubus_ams_numbers = 0; // 在线 AMS 计数
    out[0] = 0x33;       // 魔数
    out[1] = buf[1];     // 标志位（从请求包复制）
    out[4] = 0x01;       // 命令号：心跳

    uint8_t *EQPT_data_ptr = out + 6; // 设备数据起始位置

    // 遍历所有 AMS 设备，在线设备信息写入回复包
    for (uint8_t i = 0; i < ams_max_number; i++)
    {
        if (ams[i].online == true)
        {
            EQPT_data_ptr[0] = i << 2;    // AMS 地址（左移 2 位编码）
            EQPT_data_ptr[1] = ams[i].ams_type; // AMS 类型
            EQPT_data_ptr += 2;
            ahubus_ams_numbers++;
        }
    }

    out[5] = ahubus_ams_numbers; // 填入在线 AMS 数量

    // 如果 AMS 数量为偶数，添加填充字节确保数据对齐
    if ((ahubus_ams_numbers & 0x01) == 0)
    {
        EQPT_data_ptr[0] = 0x00;
        EQPT_data_ptr[1] = 0x00;
        EQPT_data_ptr += 2;
    }

    // 计算长度字段（32 位字数 - 2）
    out[2] = ((EQPT_data_ptr - out) >> 2) - 2;
    bus_port_to_host.send_data_len = ahubus_package_add_crc(out); // 添加 CRC 并设置发送长度
}

/**
 * @brief AHUB 查询命令子类型枚举
 * 
 * 定义了所有支持的查询类型，主控通过这些类型请求 AMS 返回不同类别的信息。
 */
enum class ahubus_query_type : uint8_t
{
    ams_name = 0x01,        /**< 查询 AMS 设备名称（8 字节） */
    filament_info = 0x02,   /**< 查询所有 4 个通道的耗材详细信息（ID、名称、颜色等） */
    filament_stu = 0x04,    /**< 查询所有 4 个通道的耗材实时状态（运动、温湿度、烘干等） */
    dryer_stu = 0x05,       /**< 查询所有 4 个通道的烘干机状态 */
    all_filament_stu = 0x06,/**< 查询所有 AMS 设备的所有通道状态（批量查询） */
};

/**
 * @brief AHUB 查询命令包头结构体
 * 
 * 所有 AHUB 查询回复包的通用头部格式。
 */
struct ahubus_package_query_head
{
    uint8_t magic_byte;         /**< 魔数 0x33 */
    uint8_t flag;              /**< 帧标志 */
    uint8_t length;            /**< 长度（32 位字数 - 2） */
    uint8_t crc8;             /**< CRC8 校验 */
    uint8_t command;          /**< 命令号 0x02（查询回复） */
    ahubus_query_type query_type; /**< 查询子类型 */
    uint8_t query_adr;        /**< 查询地址（原始值） */
    uint8_t data_struct_count; /**< 数据结构计数或填充标记 */
    uint8_t data[0];          /**< 柔性数组，有效载荷起始位置 */
} __attribute__((packed));

/**
 * @brief AHUB 查询回复包的默认初始化模板
 * 
 * 包含固定的魔数、标志和命令号，其他字段在运行时填充。
 */
const ahubus_package_query_head ahubus_host_package_query_init = {
    .magic_byte = 0x33,  // 魔数
    .flag = 0x80,        // 标志（回复包）
    .command = 0x02,     // 命令号：查询回复
};

/**
 * @brief 将单个耗材的状态打包为 8 字节格式
 * 
 * 将耗材的各种状态信息压缩为 8 字节的紧凑格式：
 * - 字节 0: 运动状态（bit7=在线标志, bit6:0=运动状态）
 * - 字节 1: 密封状态
 * - 字节 2: 料仓温度（有符号，°C）
 * - 字节 3: 料仓湿度（%）
 * - 字节 4: 烘干功率
 * - 字节 5: 烘干温度
 * - 字节 6-7: 剩余烘干时间（16 位，小端序）
 * 
 * @param dst 输出缓冲区（至少 8 字节）
 * @param f 指向耗材数据结构体
 */
static inline __attribute__((always_inline)) void ahub_pack_filament_stu8(uint8_t* dst, const _filament* f)
{
    uint8_t m = (uint8_t)f->motion;
    if (f->online) m |= 0x80u; // bit7 标记在线状态

    dst[0] = m;                              // 运动状态 + 在线标志
    dst[1] = f->seal_status;                 // 密封状态
    dst[2] = (uint8_t)f->compartment_temperature; // 料仓温度
    dst[3] = f->compartment_humidity;        // 料仓湿度
    dst[4] = f->dryer_power;                 // 烘干功率
    dst[5] = (uint8_t)f->dryer_temperature; // 烘干温度

    uint16_t t;
    memcpy(&t, &f->dryer_time_left, sizeof(t)); // 剩余烘干时间
    dst[6] = (uint8_t)(t & 0xFFu);  // 低字节
    dst[7] = (uint8_t)(t >> 8);     // 高字节
}

/**
 * @brief 处理主控发来的查询命令
 * 
 * 根据查询类型返回相应数据：
 * 
 * - ams_name: 返回指定 AMS 的 8 字节设备名称
 * - filament_info: 返回指定 AMS 的 4 个通道共 176 字节耗材详细信息（每通道 44 字节）
 * - filament_stu: 返回指定 AMS 的 4 个通道共 32 字节实时状态（每通道 8 字节）
 * - dryer_stu: 返回指定 AMS 的 4 个通道共 16 字节烘干机状态（每通道 4 字节）
 * - all_filament_stu: 返回所有在线 AMS 的所有通道状态（每 AMS 10 字节）
 * 
 * @param buf 接收到的查询命令数据
 */
void ahubus_slave_get_package_query(uint8_t *buf)
{
    if (bus_port_to_host.send_data_len != 0) return;

    uint8_t* out = bus_port_to_host.tx_build_buf();

    const ahubus_query_type query_type = (ahubus_query_type)buf[5]; // 查询类型
    const uint8_t query_adr_raw = buf[6];                           // 原始查询地址
    uint8_t query_adr = query_adr_raw;

    // 复制回复包头模板
    memcpy(out, &ahubus_host_package_query_init, sizeof(ahubus_package_query_head));
    out[5] = (uint8_t)query_type;   // 查询类型
    out[6] = query_adr_raw;         // 原始地址

    uint8_t *data_ptr = out + 4;    // 数据载荷起始位置

#ifdef xMCU
    // xMCU 模式：地址高 4 位为 AMS 编号
    query_adr = (uint8_t)(query_adr >> 4);
#endif

    // 批量查询不需要地址范围检查
    if (query_type != ahubus_query_type::all_filament_stu)
    {
        if (query_adr >= ams_max_number) return; // 地址越界
    }

    switch (query_type)
    {
    case ahubus_query_type::ams_name:
        // 查询 AMS 名称：复制 8 字节设备名称
        memcpy(data_ptr + 4, ams[query_adr].name, 8);
        out[2] = 2;       // 长度（32 位字数）
        out[7] = 0x01;    // 数据结构计数
        break;

    case ahubus_query_type::filament_info:
        // 查询耗材信息：复制 4 个通道共 176 字节（每通道 44 字节）
        memcpy(data_ptr + 4,   ams[query_adr].filament[0].bambubus_filament_id, 44);
        memcpy(data_ptr + 48,  ams[query_adr].filament[1].bambubus_filament_id, 44);
        memcpy(data_ptr + 92,  ams[query_adr].filament[2].bambubus_filament_id, 44);
        memcpy(data_ptr + 136, ams[query_adr].filament[3].bambubus_filament_id, 44);
        out[2] = 44;      // 长度
        out[7] = 0x01;
        break;

    case ahubus_query_type::filament_stu:
    {
        // 查询耗材实时状态：每通道 8 字节，共 32 字节
        bus_now_ams_num = query_adr; // 更新当前活动 AMS 编号
        ahub_pack_filament_stu8(data_ptr + 4,  &ams[query_adr].filament[0]);
        ahub_pack_filament_stu8(data_ptr + 12, &ams[query_adr].filament[1]);
        ahub_pack_filament_stu8(data_ptr + 20, &ams[query_adr].filament[2]);
        ahub_pack_filament_stu8(data_ptr + 28, &ams[query_adr].filament[3]);
        out[2] = 8;
        out[7] = 0x01;
        break;
    }

    case ahubus_query_type::dryer_stu:
        // 查询烘干机状态：每通道 4 字节（功率+温度+时间），共 16 字节
        memcpy(data_ptr + 4,  &(ams[query_adr].filament[0].dryer_power), 4);
        memcpy(data_ptr + 8,  &(ams[query_adr].filament[1].dryer_power), 4);
        memcpy(data_ptr + 12, &(ams[query_adr].filament[2].dryer_power), 4);
        memcpy(data_ptr + 16, &(ams[query_adr].filament[3].dryer_power), 4);
        out[2] = 4;
        out[7] = 0x01;
        break;

    case ahubus_query_type::all_filament_stu:
    {
        // 批量查询所有 AMS 的所有通道状态
        uint8_t *ams_filament_data_ptr = data_ptr + 4;
        uint8_t ams_data_count = 0; // 在线 AMS 计数

        for (uint8_t i = 0; i < ams_max_number; i++)
        {
            if (!ams[i].online) continue; // 跳过离线 AMS

#ifdef xMCU
            ams_filament_data_ptr[0] = (uint8_t)(i << 4); // xMCU 模式：左移 4 位编码
#else
            ams_filament_data_ptr[0] = i; // 简单编号模式
#endif
            ams_filament_data_ptr[1] = ams[i].online; // 在线状态

            // 写入 4 个通道的运动状态和密封状态
            ams_filament_data_ptr[2] = (uint8_t)ams[i].filament[0].motion;
            ams_filament_data_ptr[3] = (uint8_t)ams[i].filament[0].seal_status;
            ams_filament_data_ptr[4] = (uint8_t)ams[i].filament[1].motion;
            ams_filament_data_ptr[5] = (uint8_t)ams[i].filament[1].seal_status;
            ams_filament_data_ptr[6] = (uint8_t)ams[i].filament[2].motion;
            ams_filament_data_ptr[7] = (uint8_t)ams[i].filament[2].seal_status;
            ams_filament_data_ptr[8] = (uint8_t)ams[i].filament[3].motion;
            ams_filament_data_ptr[9] = (uint8_t)ams[i].filament[3].seal_status;

            // 在线标志编码到运动状态的 bit7
            if (ams[i].filament[0].online) ams_filament_data_ptr[2] |= 0x80u;
            if (ams[i].filament[1].online) ams_filament_data_ptr[4] |= 0x80u;
            if (ams[i].filament[2].online) ams_filament_data_ptr[6] |= 0x80u;
            if (ams[i].filament[3].online) ams_filament_data_ptr[8] |= 0x80u;

            ams_filament_data_ptr += 10; // 每个 AMS 占 10 字节
            ams_data_count++;
        }

        // 填充至 4 字节对齐
        while (((uintptr_t)(ams_filament_data_ptr - data_ptr) & 3u) != 0u) {
            *ams_filament_data_ptr++ = 0x00;
        }

        // 计算长度和数据计数
        out[2] = (uint8_t)(((ams_filament_data_ptr - data_ptr) >> 2) - 1u);
        out[7] = ams_data_count;
        break;
    }

    default:
        return; // 未知查询类型，不回复
    }

    bus_port_to_host.send_data_len = ahubus_package_add_crc(out); // 添加 CRC 并设置发送长度
}


/**
 * @brief AHUB 同步请求列表结构体
 * 
 * 用于批量同步多个 AMS 通道的状态信息。
 */
struct ahubus_sync_req_list_struct
{
    uint8_t ams_num;            /**< AMS 编号 */
    uint8_t filament_channel;   /**< 耗材通道号 */
    ahubus_set_type type;       /**< 同步类型 */
};

/**
 * @brief AHUB 设置命令包头结构体
 * 
 * 所有 AHUB 设置回复包的通用头部格式。
 */
struct ahubus_package_set_head
{
    uint8_t magic_byte;     /**< 魔数 0x33 */
    uint8_t flag;          /**< 帧标志 */
    uint8_t length;        /**< 长度（32 位字数 - 2） */
    uint8_t crc8;         /**< CRC8 校验 */
    uint8_t command;      /**< 命令号 0x03（设置回复） */
    uint8_t set_type;     /**< 设置子类型 */
    uint8_t set_adr;      /**< 设置地址（原始值） */
    uint8_t data_struct_count; /**< 数据结构计数 */
    uint8_t data[0];      /**< 柔性数组，有效载荷起始位置 */
} __attribute__((packed));

/**
 * @brief AHUB 设置回复包的默认初始化模板
 */
const ahubus_package_set_head ahubus_host_package_set_init = {
    .magic_byte = 0x33,  // 魔数
    .flag = 0x80,        // 标志（回复包）
    .command = 0x03,     // 命令号：设置回复
};

/**
 * @brief 处理主控发来的设置命令
 * 
 * 根据设置类型执行不同的写入操作：
 * 
 * - filament_info: 设置指定通道的耗材信息（ID、颜色、温度、名称）
 *   数据布局：data[48]=通道号, data[4..47]=44字节耗材信息
 * 
 * - dryer_stu: 设置指定通道的烘干机状态
 *   数据布局：data[8]=通道号, data[4..7]=4字节烘干状态
 * 
 * - all_filament_stu: 批量设置所有 AMS 的通道运动状态
 *   每个 AMS 6 字节：地址 + 当前通道号 + 4个通道的运动状态
 * 
 * 设置完成后发送一个空的确认回复包。
 * 
 * @param buf 接收到的设置命令数据
 */
void ahubus_slave_get_package_set(uint8_t *buf)
{
    if (bus_port_to_host.send_data_len != 0) return;

    uint8_t* out = bus_port_to_host.tx_build_buf();

    const uint8_t set_type_raw = buf[5];  // 设置类型（原始值）
    const uint8_t set_adr_raw  = buf[6];  // 设置地址（原始值）

    ahubus_set_type set_type = (ahubus_set_type)set_type_raw;
    uint8_t set_adr = set_adr_raw;

    uint8_t *data_ptr = buf + 4; // 数据载荷起始位置

#ifdef xMCU
    // xMCU 模式：地址高 4 位为 AMS 编号
    set_adr = (uint8_t)(set_adr >> 4);
#endif

    if (set_adr >= ams_max_number) return; // 地址越界

    switch (set_type)
    {
    case ahubus_set_type::filament_info:
    {
        // 设置耗材信息
        const uint8_t filament_channel = data_ptr[48]; // 通道号在 data[48]
        if (filament_channel >= 4) return;              // 通道号越界
        // 复制 44 字节耗材信息（ID、颜色、温度、名称等）
        memcpy(&(ams[set_adr].filament[filament_channel].bambubus_filament_id), data_ptr + 4, 44);

        // 如果是当前 AMS，触发保存操作
        if (set_adr == (uint8_t)BAMBU_BUS_AMS_NUM)
            ams_datas_set_need_to_save_filament(filament_channel);

        break;
    }
    case ahubus_set_type::dryer_stu:
    {
        // 设置烘干机状态
        const uint8_t dryer_channel = data_ptr[8]; // 通道号在 data[8]
        if (dryer_channel >= 4) return;
        // 复制 4 字节烘干状态（功率、温度、时间）
        memcpy(&(ams[set_adr].filament[dryer_channel].dryer_power), data_ptr + 4, 4);
        break;
    }
    case ahubus_set_type::all_filament_stu:
    {
        // 批量设置所有 AMS 的通道运动状态
        const uint8_t data_struct_count = buf[7]; // 数据结构数量
        uint8_t *data_struct_ptr = data_ptr + 4;

        for (uint8_t i = 0; i < data_struct_count; i++)
        {
            uint8_t ams_adr = data_struct_ptr[0]; // AMS 地址
#ifdef xMCU
            ams_adr = (uint8_t)(ams_adr >> 4);    // xMCU 模式：右移 4 位
#endif
            if (ams_adr >= ams_max_number) { data_struct_ptr += 6; continue; } // 跳过无效地址

            // 更新当前通道号和 4 个通道的运动状态
            ams[ams_adr].now_filament_num = data_struct_ptr[1];
            ams[ams_adr].filament[0].motion = (_filament_motion)(data_struct_ptr[2] & 0x7Fu); // bit7 为在线标志，屏蔽
            ams[ams_adr].filament[1].motion = (_filament_motion)(data_struct_ptr[3] & 0x7Fu);
            ams[ams_adr].filament[2].motion = (_filament_motion)(data_struct_ptr[4] & 0x7Fu);
            ams[ams_adr].filament[3].motion = (_filament_motion)(data_struct_ptr[5] & 0x7Fu);

            data_struct_ptr += 6; // 每个 AMS 数据结构 6 字节
        }
        break;
    }
    default:
        return; // 未知设置类型，不回复
    }

    // 构建确认回复包（空数据）
    memcpy(out, &ahubus_host_package_set_init, sizeof(ahubus_package_set_head));
    out[5] = set_type_raw;           // 设置类型
    out[6] = set_adr_raw;            // 设置地址
    out[7] = 0;                      // 数据结构计数（回复为空）
    out[2] = 0;                      // 长度

    bus_port_to_host.send_data_len = ahubus_package_add_crc(out);
}

/**
 * @brief AHUB 协议主运行函数
 * 
 * 在主循环中周期性调用，处理 AHUB 总线上的所有通信。
 * 
 * 处理流程：
 * 1. 从总线端口读取接收到的数据包（临界区保护）
 * 2. 验证包的有效性（魔数 0x33、CRC32 校验）
 * 3. 解析包类型并分发到对应的处理函数
 * 4. 清空接收缓冲区
 * 5. 检测心跳超时（1000ms 超时）
 * 
 * @return ahubus_package_type 当前处理的状态
 *   - heartbeat/query/set: 对应的数据包类型
 *   - error: 心跳超时
 *   - none: 无有效数据包
 */
ahubus_package_type ahubus_run()
{
    ahubus_package_type package_type = ahubus_package_type::none;

    static uint32_t deadline = 0;   // 心跳截止时间
    const uint32_t now = time_ticks32();

    int rx_len = 0;
    _bus_data_type t = _bus_data_type::none;
    uint8_t *buf = nullptr;

    // 临界区：读取接收数据（避免与中断竞争）
    {
        const uint32_t s = irq_save_wch();
        rx_len = bus_port_to_host.recv_data_len;
        t      = bus_port_to_host.bus_package_type;
        buf    = bus_port_to_host.bus_recv_data_ptr;
        irq_restore_wch(s);
    }

    // 检查是否收到有效的 AHUB 数据包
    if (rx_len > 0 && t == _bus_data_type::ahub_bus)
    {
        if (buf != nullptr && rx_len <= 1280 && buf[0] == 0x33)
        {
            package_type = ahubus_get_package_type(buf); // 验证 CRC 并获取包类型

            switch (package_type)
            {
            case ahubus_package_type::heartbeat:
                ahubus_slave_get_package_heartbeat(buf); // 处理心跳包
                deadline = now + ms_to_ticks32(1000u);    // 刷新心跳截止时间
                break;

            case ahubus_package_type::query:
                ahubus_slave_get_package_query(buf);      // 处理查询命令
                break;

            case ahubus_package_type::set:
                ahubus_slave_get_package_set(buf);         // 处理设置命令
                break;

            default:
                break;
            }
        }

        // 临界区：清空接收缓冲区
        {
            const uint32_t s = irq_save_wch();
            bus_port_to_host.recv_data_len    = 0;
            bus_port_to_host.bus_package_type = _bus_data_type::none;
            irq_restore_wch(s);
        }
    }

    // 心跳超时检测：当前时间超过截止时间
    if ((int32_t)(now - deadline) > 0)
        package_type = ahubus_package_type::error;

    return package_type;
}
