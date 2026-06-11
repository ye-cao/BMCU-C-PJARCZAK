#pragma once
#include <stdint.h>

/**
 * @brief AMS单元最大数量
 *
 * BMCU-C系统最多支持4个AMS单元，每个AMS单元有4个料槽，共16个耗材位。
 * ams[]全局数组大小即为该宏定义值。
 */
#define ams_max_number 4

/**
 * @brief 耗材运动状态枚举
 *
 * 描述当前耗材在AMS系统中的运动/工作状态。
 * 与上位机通信时用于表示料槽当前所处阶段，对应不同的控制指令。
 */
enum class _filament_motion : uint8_t
{
    idle            = 0,  /**< 空闲状态，耗材静止，无任何动作 */
    send_out        = 1,  /**< 送出状态，耗材正在从料槽向外送出（送料） */
    on_use          = 2,  /**< 使用中状态，耗材已送达打印头，正在打印使用 */
    before_pull_back= 3,  /**< 回抽前状态，准备将耗材从打印头回抽（过渡阶段） */
    pull_back       = 4,  /**< 回抽中状态，耗材正在从打印头回抽至AMS料槽 */
    before_on_use   = 5,  /**< 使用前状态，耗材即将进入使用阶段（对应指令 09 A5） */
    stop_on_use     = 6   /**< 停止使用状态，耗材停止使用/退出打印（对应指令 07 00） */
};

/**
 * @brief 单个耗材信息结构体
 *
 * 表示一个料槽中的一卷耗材，包含耗材参数信息、烘干器状态、运行时状态。
 * 该结构体通过packed+aligned(4)确保内存布局紧凑且关键字段4字节对齐，
 * 便于总线通信数据的直接映射和DMA传输。
 */
struct _filament
{
    /* ==================== 耗材参数信息字段 ==================== */

    /**
     * BambuLab总线耗材ID字符串
     * 用于在BambuLab生态系统中唯一标识耗材类型，默认值为"GFG00"。
     * 长度8字节，末尾包含'\0'终止符。
     */
    char bambubus_filament_id[8] = "GFG00";

    /** 耗材颜色 - 红色分量 (0x00~0xFF)，默认白色(0xFF) */
    uint8_t color_R = 0xFF;
    /** 耗材颜色 - 绿色分量 (0x00~0xFF)，默认白色(0xFF) */
    uint8_t color_G = 0xFF;
    /** 耗材颜色 - 蓝色分量 (0x00~0xFF)，默认白色(0xFF) */
    uint8_t color_B = 0xFF;
    /** 耗材颜色 - Alpha通道/透明度 (0x00~0xFF)，默认不透明(0xFF) */
    uint8_t color_A = 0xFF;

    /** 耗材打印最低温度(℃)，默认220℃ */
    int16_t temperature_min = 220;
    /** 耗材打印最高温度(℃)，默认240℃ */
    int16_t temperature_max = 240;

    /**
     * 耗材名称字符串
     * 例如"PETG"、"PLA"、"ABS"等，长度20字节，用于显示和通信。
     */
    char name[20] = "PETG";

    /**
     * XHub唯一标识ID
     * 用于标识该耗材与XHub设备的绑定关系，0表示未绑定。
     */
    uint64_t xhub_unique_id = 0;

    /* ==================== 耗材参数信息字段结束 ==================== */

    /* ==================== 耗材烘干器状态字段 ==================== */

    /** 烘干器设定功率(W)，0表示烘干器未启用 */
    uint8_t dryer_power = 0;
    /** 烘干器设定温度(℃)，有符号，正值表示加热温度 */
    int8_t dryer_temperature = 0;
    /** 烘干器剩余工作时间(分钟)，向上取整，0表示未在烘干 */
    uint16_t dryer_time_left = 0;

    /* ==================== 耗材烘干器状态字段结束 ==================== */

    /* ==================== 耗材状态信息字段 ==================== */

    /**
     * 耗材在线状态
     * true=料槽中有耗材且被系统识别，false=料槽为空或未识别。
     * 上位机通过此字段判断哪些料槽可用。
     */
    bool online = true;

    /**
     * 耗材当前运动状态
     * 枚举类型_filament_motion，表示耗材当前处于哪个运动阶段。
     */
    _filament_motion motion = _filament_motion::idle;

    /**
     * 密封结构状态
     * 0: 无密封结构（普通料槽）
     * 1: 已开盖（密封盖已打开，允许取放耗材）
     * 2: 已合盖（密封盖已关闭，密封保存耗材）
     */
    uint8_t seal_status = 0;

    /**
     * 料槽内腔温度(℃)
     * 有符号8位整数，范围-128℃~127℃，默认22℃。
     * 用于监控耗材存储环境温度。
     */
    int8_t compartment_temperature = 22;

    /**
     * 料槽内腔湿度(%)
     * 无符号8位整数，范围0%~100%，默认20%。
     * 用于监控耗材存储环境湿度，防止耗材受潮。
     */
    uint8_t compartment_humidity = 20;

    /**
     * 耗材已使用长度(米)
     * 4字节对齐，用于累计统计该耗材的使用量。
     * 上位机可据此显示耗材剩余量百分比。
     */
    float meters __attribute__((aligned(4))) = 1.0f;

    /**
     * 虚拟计数器(米)
     * 4字节对齐，用于辅助计数或临时存储的虚拟耗材长度值。
     */
    float meters_virtual_count __attribute__((aligned(4))) = 0.0f;

    /**
     * @brief 初始化单个耗材的所有字段为默认值
     *
     * 将耗材ID设为"GFG00"，颜色设为白色，温度范围220~240℃，
     * 名称设为"PETG"，运动状态设为idle，环境参数设为默认值。
     * 注意：该函数不修改seal_status字段（保留原值）。
     */
    void init()
    {
        xhub_unique_id = 0;
        bambubus_filament_id[0] = 'G';
        bambubus_filament_id[1] = 'F';
        bambubus_filament_id[2] = 'G';
        bambubus_filament_id[3] = '0';
        bambubus_filament_id[4] = '0';
        bambubus_filament_id[5] = '\0';
        color_R = 0xFF;
        color_G = 0xFF;
        color_B = 0xFF;
        color_A = 0xFF;
        temperature_min = 220;
        temperature_max = 240;
        name[0] = 'P';
        name[1] = 'E';
        name[2] = 'T';
        name[3] = 'G';
        name[4] = '\0';
        meters = 1;
        meters_virtual_count = 0;
        online = true;
        motion = _filament_motion::idle;
        compartment_temperature = 22;
        compartment_humidity = 20;
    }

} __attribute__((packed, aligned(4)));

/**
 * @brief 编译期断言：meters字段必须4字节对齐
 *
 * 确保meters字段在结构体内的偏移量是4的倍数，
 * 以满足RISC-V对浮点数访问的对齐要求，避免硬件异常。
 */
static_assert((__builtin_offsetof(_filament, meters) & 3u) == 0u, "meters misaligned");

/**
 * @brief 编译期断言：meters_virtual_count字段必须4字节对齐
 *
 * 确保meters_virtual_count字段在结构体内的偏移量是4的倍数，
 * 以满足RISC-V对浮点数访问的对齐要求。
 */
static_assert((__builtin_offsetof(_filament, meters_virtual_count) & 3u) == 0u, "meters_virtual_count misaligned");


/**
 * @brief AMS单元结构体
 *
 * 表示一个AMS（Automatic Material System，自动供料系统）单元。
 * 每个AMS单元包含4个耗材料槽，支持4种不同颜色/类型的耗材。
 * BMCU-C系统最多管理4个这样的AMS单元（共16个料槽）。
 * aligned(4)确保结构体4字节对齐，便于总线通信数据传输。
 */
struct _ams
{
    /**
     * AMS类型标识
     * 0: 标准AMS单元
     * 其他值: 不同型号或配置的AMS单元（如AMS Lite等）
     */
    uint8_t ams_type = 0;

    /**
     * 该AMS单元包含的4个耗材料槽
     * 每个元素是一个_filament结构体，代表一个独立的耗材位。
     * 索引0~3分别对应料槽1~4。
     */
    _filament filament[4];

    /**
     * 当前正在使用的耗材料槽编号
     * 0~3: 对应当前正在送料/打印的料槽索引
     * 0xFF: 无耗材在使用中（空闲状态）
     */
    uint8_t now_filament_num = 0xFF;

    /**
     * AMS单元名称
     * 长度8字节，用于在UI中显示或通信中标识该AMS单元。
     */
    char name[8];

    /**
     * 耗材使用标志位
     * 每个bit对应一个料槽(0~3)的使用状态。
     * bit0=料槽1, bit1=料槽2, bit2=料槽3, bit3=料槽4。
     * 1表示该料槽正在被使用/选中，0表示未使用。
     */
    uint8_t filament_use_flag = 0;

    /**
     * 送料压力值
     * 0xFFFF: 默认值/无压力数据（传感器未就绪或未连接）
     * 其他值: 实际检测到的送料压力值，用于判断是否堵料或断料。
     * 压力异常时可触发保护机制停止送料。
     */
    uint16_t pressure = 0xFFFF;

    /**
     * AMS单元在线状态
     * true: 该AMS单元已连接并可通信
     * false: 该AMS单元未连接或通信异常
     */
    bool online = false;

    /**
     * @brief 初始化整个AMS单元
     *
     * 将当前使用耗材编号设为0xFF（无），使用标志位清零，
     * 压力设为默认值0xFFFF，离线状态，然后依次初始化所有4个耗材料槽。
     * 该函数在系统启动或复位时调用。
     */
    void init()
    {
        now_filament_num = 0xFF;
        filament_use_flag = 0;
        pressure = 0xFFFF;
        online = false;
        for (uint8_t i = 0; i < 4; i++)
        {
            filament[i].init();
        }
    }
} __attribute__((aligned(4)));

/**
 * 全局AMS单元数组
 * 最多包含ams_max_number(4)个AMS单元，每个AMS有4个料槽。
 * 所有AMS相关的数据操作都基于此数组进行。
 */
extern _ams ams[ams_max_number];

/**
 * @brief 初始化所有AMS单元
 *
 * 遍历ams数组，依次调用每个AMS单元的init()方法，
 * 将所有AMS及其耗材数据重置为默认值。
 * 在系统启动时调用一次。
 */
extern void ams_init();

/**
 * @brief 当前正在总线上通信的AMS单元编号
 *
 * 标识当前正在通过BambuLab总线进行数据交换的AMS单元索引(0~3)。
 * 在多AMS系统中，总线通信是分时进行的，该变量记录当前轮询到哪个AMS。
 * 用于区分总线响应数据属于哪个AMS单元。
 */
extern uint8_t bus_now_ams_num;
