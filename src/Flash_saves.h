#pragma once
#include <stdint.h>

/**
 * @file Flash_saves.h
 * @brief Flash 非易失性存储(NVM)管理接口
 *
 * 使用 CH32V203C8T6 最后一个 4KB Flash 扇区 (0x0800F000 - 0x0800FFFF)
 * 来持久化存储 AMS 耗材信息、运动参数和电机校准数据。
 *
 * 存储布局:
 *   0x000-0x0FF : 校准数据 (CAL)  1页 256字节
 *   0x100-0x1FF : 运动参数 (MOT)  1页 256字节
 *   0x200-0x5FF : 耗材信息 (FIL)  4页 各256字节 (每槽位一个耗材)
 *   0x600-0xFFF : 状态记录 (STA)  10页 环形日志
 */

#ifndef BAMBU_BUS_AMS_NUM
#define BAMBU_BUS_AMS_NUM 0
#endif

/** @brief NVM 基地址: Flash 最后 4KB 扇区起始位置 (CH32V203C8 总容量 64KB) */
#define FLASH_NVM_BASE_ADDR   ((uint32_t)0x0800F000)   // 4KB sector

/** @brief 校准数据页地址 (1页 256字节) */
#define FLASH_NVM_CAL_ADDR    (FLASH_NVM_BASE_ADDR + 0x000) // 1x256B
/** @brief 运动参数页地址 (1页 256字节) */
#define FLASH_NVM_MOTION_ADDR (FLASH_NVM_BASE_ADDR + 0x100) // 1x256B
/** @brief 耗材信息页组地址 (4页 各256字节, 索引0~3, 范围到 +0x5FF) */
#define FLASH_NVM_AMS_ADDR    (FLASH_NVM_BASE_ADDR + 0x200) // 4x256B (0..3) => do +0x5FF

/** @brief 每页大小 256 字节 (CH32V203 Flash 最小可编程单位) */
#define FLASH_NVM256_PAGE_SIZE (256u)
/** @brief NVM 总大小 4096 字节 (一个完整扇区) */
#define FLASH_NVM_TOTAL_SIZE    (4096u)
/** @brief NVM 页总数 (4096 / 256 = 16) */
#define FLASH_NVM_PAGE_COUNT    (FLASH_NVM_TOTAL_SIZE / FLASH_NVM256_PAGE_SIZE)
/** @brief 页内 CRC32 偏移位置: 前 252 字节用于数据, 最后 4 字节存 CRC */
#define NVM256_CRC_OFF         (252u)

/** @brief 耗材数据魔数 'FIL1' 用于数据完整性校验 */
static constexpr uint32_t MAGIC_FIL = 0x314C4946u; // 'FIL1'
/** @brief 校准数据魔数 'CAL2' */
static constexpr uint32_t MAGIC_CAL = 0x324C4143u; // 'CAL2'
/** @brief 运动参数魔数 'MOT1' */
static constexpr uint32_t MAGIC_MOT = 0x31544F4Du; // 'MOT1'
/** @brief 状态记录魔数 'STA1' */
static constexpr uint32_t MAGIC_STA = 0x31415453u; // 'STA1'
/** @brief 数据版本号 0x0001 */
static constexpr uint16_t VER_1     = 0x0001u;

/**
 * @brief NVM 256字节页通用头部结构
 *
 * 每个 256 字节页都以此头部开头, 用于标识数据类型、版本和载荷长度。
 * 最后 4 字节 (偏移 252) 存储整个前 252 字节的 CRC32 校验值。
 */
struct __attribute__((packed, aligned(4))) NVM256_HDR
{
    uint32_t magic;  /**< 魔数标识, 用于识别数据类型 (如 MAGIC_FIL, MAGIC_CAL 等) */
    uint16_t ver;    /**< 数据格式版本号 */
    uint16_t len;    /**< 载荷数据长度 (不含头部和CRC) */
    uint32_t rsv;    /**< 保留字段, 可用于存储额外标志 (如校准极性) */
};

/**
 * @brief 耗材信息紧凑存储结构 (32字节)
 *
 * 对应 BambuLab AMS 系统中的单个耗材槽位信息。
 * 包含耗材 ID、RGBW 颜色、温度范围和名称。
 * 此结构会被打包写入 Flash 耗材槽位中。
 */
struct __attribute__((packed, aligned(4))) Flash_FilamentInfo
{
    uint8_t  bambubus_filament_id[8]; /**< BambuLab 总线协议中的耗材唯一 ID (8字节) */
    uint8_t  color_R;                 /**< 耗材颜色红色分量 (0-255) */
    uint8_t  color_G;                 /**< 耗材颜色绿色分量 (0-255) */
    uint8_t  color_B;                 /**< 耗材颜色蓝色分量 (0-255) */
    uint8_t  color_A;                 /**< 耗材颜色 Alpha/白色分量 (0-255) */
    uint16_t temperature_min;         /**< 最低打印温度 (摄氏度) */
    uint16_t temperature_max;         /**< 最高打印温度 (摄氏度) */
    char     name[16];                /**< 耗材名称 (UTF-8, 最多15字符+NUL终止符) */
};

/**
 * @brief 初始化 Flash NVM 子系统
 *
 * 使能 CRC 硬件时钟, 清空运行时缓存,
 * 并加载全部 4 个耗材槽位的缓存数据。
 */
void Flash_saves_init(void);

/**
 * @brief 从 Flash 读取指定耗材槽位的信息 (从缓存读取)
 * @param filament_idx 耗材索引 (0-3)
 * @param out 输出: 耗材信息结构体指针
 * @return true 读取成功, false 索引无效或无数据
 */
bool Flash_AMS_filament_read(uint8_t filament_idx, Flash_FilamentInfo* out);

/**
 * @brief 将耗材信息写入 Flash 指定槽位
 *
 * 采用追加写入策略: 在当前页内按槽位顺序写入,
 * 页满时自动擦除并从头开始。写入前会与缓存比较,
 * 若数据相同则跳过写入以减少 Flash 磨损。
 *
 * @param filament_idx 耗材索引 (0-3)
 * @param info 输入: 要写入的耗材信息
 * @return true 写入成功
 */
bool Flash_AMS_filament_write(uint8_t filament_idx, const Flash_FilamentInfo* info);

/**
 * @brief 清除指定耗材槽位的全部 Flash 数据
 * @param filament_idx 耗材索引 (0-3)
 * @return true 擦除成功
 */
bool Flash_AMS_filament_clear(uint8_t filament_idx);

/**
 * @brief 读取 AMS 已加载耗材的通道状态
 *
 * 从环形状态日志中找到最新一条有效记录,
 * 返回当时正在使用的耗材通道号。
 *
 * @param loaded_ch 输出: 已加载的耗材通道 (0-3), 0xFF 表示无数据
 * @return true 读取成功 (即使无有效记录也会返回 true)
 */
bool Flash_AMS_state_read(uint8_t* loaded_channel);

/**
 * @brief 写入 AMS 已加载耗材通道状态
 *
 * 使用环形日志机制写入, 每次写入自动递增序列号。
 * 若当前通道与上次写入相同则跳过写入。
 * 写入失败时自动擦除当前页并重试。
 *
 * @param loaded_ch 当前已加载的耗材通道 (0-3)
 * @return true 写入成功
 */
bool Flash_AMS_state_write(uint8_t loaded_channel);

/**
 * @brief 读取电机拉动校准数据
 * @param offs[4] 输出: 4通道 ADC 偏移量
 * @param vmin[4] 输出: 4通道 ADC 最小电压值
 * @param vmax[4] 输出: 4通道 ADC 最大电压值
 * @param pol[4] 输出: 4通道极性 (1=正, -1=负), 传 NULL 则不返回
 * @return true 读取成功
 */
bool Flash_MC_PULL_cal_read(float offs[4], float vmin[4], float vmax[4], int8_t pol[4]);

/**
 * @brief 写入全部电机拉动校准数据
 * @param offs[4] 4通道 ADC 偏移量
 * @param vmin[4] 4通道 ADC 最小电压值
 * @param vmax[4] 4通道 ADC 最大电压值
 * @param pol[4] 4通道极性 (1=正, -1=负), 传 NULL 则极性全记为正
 * @return true 写入成功
 */
bool Flash_MC_PULL_cal_write_all(const float offs[4], const float vmin[4], const float vmax[4], const int8_t pol[4]);

/**
 * @brief 清除电机拉动校准数据 (擦除校准页)
 * @return true 擦除成功
 */
bool Flash_MC_PULL_cal_clear(void);

/**
 * @brief 完全擦除整个 NVM 4KB Flash 扇区
 *
 * 会清空所有数据: 校准、运动、耗材和状态记录,
 * 同时重置全部运行时缓存。
 *
 * @return true 擦除成功
 */
bool Flash_NVM_full_clear(void);

/**
 * @brief 从 Flash 读取运动参数 (通用载荷)
 * @param out 输出缓冲区
 * @param bytes 最大读取字节数
 * @return true 读取成功
 */
bool Flash_Motion_read(void* out, uint16_t bytes);

/**
 * @brief 将运动参数写入 Flash (通用载荷)
 * @param in 输入数据缓冲区
 * @param bytes 要写入的字节数
 * @return true 写入成功
 */
bool Flash_Motion_write(const void* in, uint16_t bytes);

/**
 * @brief 清除运动参数 (擦除运动页)
 * @return true 擦除成功
 */
bool Flash_Motion_clear(void);
