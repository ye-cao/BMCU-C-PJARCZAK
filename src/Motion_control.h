/**
 * @file    Motion_control.h
 * @brief   BMCU-C 运动控制模块头文件 —— 管理4通道耗材进给/回退电机的完整运动逻辑
 *
 * 本模块是 BMCU-C 智能耗材架控制器的核心运动控制层，运行在 CH32V203C8T6 RISC-V MCU 上。
 * 主要职责：
 *   - PID 电机速度调节
 *   - AS5600 磁性旋转编码器读取（软件 I2C），用于耗材位置传感
 *   - 4通道电机 PWM 控制
 *   - 耗材进给/回退运动序列
 *   - 双微动开关检测（BMCU_DM_TWO_MICROSWITCH 编译选项）
 *   - 自动加载辅助
 *   - 在线检测逻辑（AS5600 传感器健康监控）
 *   - 运动状态：idle、send_out、on_use、before_pull_back、pull_back
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化运动控制模块
 *        设置 PWM 硬件、读取 Flash 保存的电机方向校准数据、
 *        初始化 AS5600 编码器 I2C、检测初始传感器状态
 */
void Motion_control_init();

/**
 * @brief 设置指定通道的电机 PWM 输出
 *        正值 = 正方向，负值 = 反方向，0 = 制动（两侧均输出高电平）
 * @param CHx 通道编号（0..3）
 * @param PWM PWM 值（-1000..1000），绝对值对应占空比
 */
void Motion_control_set_PWM(uint8_t CHx, int PWM);

/**
 * @brief 运动控制主循环函数，每次主循环迭代调用一次
 *        负责读取 ADC 传感器、更新 AS5600 编码器数据、执行状态机切换、
 *        运行 PID 控制器、输出 PWM、处理自动卸料和校准重置等
 * @param error 错误状态标志，非零时所有电机停止（安全保护）
 */
void Motion_control_run(int error);

/**
 * @brief 将双微动开关的"无料"电压阈值保存到 Flash
 *        用于校准自动加载检测的触发灵敏度
 * @return true 保存成功，false 保存失败
 */
bool Motion_control_save_dm_key_none_thresholds(void);

/**
 * @brief 检测4个耗材通道是否物理插入
 *        通过 ADC 读取各通道电压，判断是否在有效范围（0.3V~3.0V）内
 */
void MC_PULL_detect_channels_inserted();

/* ==================== 外部变量声明 ==================== */

/** @brief 各通道速度传感器的电压偏移补偿量（单位：V） */
extern float   MC_PULL_V_OFFSET[4];

/** @brief 各通道速度传感器的最小电压值（对应传感器行程起点） */
extern float   MC_PULL_V_MIN[4];

/** @brief 各通道速度传感器的最大电压值（对应传感器行程终点） */
extern float   MC_PULL_V_MAX[4];

/** @brief 各通道速度传感器的百分比读数（0~100，整数四舍五入） */
extern uint8_t MC_PULL_pct[4];

/** @brief 各通道传感器极性（+1 = 正常，-1 = 反向，用于兼容不同接线方向） */
extern int8_t  MC_PULL_POLARITY[4];

/** @brief 双微动开关模式下各通道的"无料"电压阈值（单位：V） */
extern float   MC_DM_KEY_NONE_THRESH[4];

/** @brief 各通道是否物理插入（true = 已插入） */
extern bool    filament_channel_inserted[4];

/* ==================== 编译时配置宏 ==================== */

/**
 * @brief BambuBus AMS 编号（0~3），通过 platformio.ini 的 -DBAMBU_BUS_AMS_NUM 指定
 *        用于选择当前 BMCU-C 控制的是哪个 AMS 单元
 */
#ifndef BAMBU_BUS_AMS_NUM
#define BAMBU_BUS_AMS_NUM 0
#endif

/**
 * @brief AMS 回退长度（单位：米），通过 platformio.ini 的 -DAMS_RETRACT_LEN 指定
 *        用于控制耗材从打印头回退到 BMCU 内的距离
 */
#ifndef AMS_RETRACT_LEN
#define AMS_RETRACT_LEN 0.2f
#endif

/**
 * @brief 双微动开关模式使能标志
 *        设为 1 时启用双微动开关检测逻辑（DM 型号专用）
 *        用于实现自动加载/卸料辅助功能
 */
#ifndef BMCU_DM_TWO_MICROSWITCH
#define BMCU_DM_TWO_MICROSWITCH 0
#endif

/**
 * @brief 在线 LED 显示耗材 RGB 颜色使能标志
 *        设为 1 时，当耗材加载成功后，ONLINE LED 会显示该耗材的 RGB 颜色
 */
#ifndef BMCU_ONLINE_LED_FILAMENT_RGB
#define BMCU_ONLINE_LED_FILAMENT_RGB 0
#endif

/** @brief 运动控制使用的 AMS 编号（内部使用） */
#ifndef motion_control_ams_num
#define motion_control_ams_num BAMBU_BUS_AMS_NUM
#endif

/** @brief 耗材回退距离（单位：米，内部使用） */
#ifndef motion_control_pull_back_distance
#define motion_control_pull_back_distance AMS_RETRACT_LEN
#endif

/** @brief 编译时校验：AMS 编号必须在 0~3 范围内 */
#if (BAMBU_BUS_AMS_NUM < 0) || (BAMBU_BUS_AMS_NUM > 3)
#error "BAMBU_BUS_AMS_NUM must be in range 0..3"
#endif
