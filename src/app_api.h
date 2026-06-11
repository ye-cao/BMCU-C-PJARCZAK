#pragma once
#include <stdint.h>
#include "ws2812.h"

/**
 * @file app_api.h
 * @brief 应用层API头文件
 * @details 提供BMCU-C固件的应用层接口函数，包括RGB LED控制和AMS(自动供料系统)状态管理。
 */

/** @brief 4通道WS2812 RGB LED对象数组(外部定义，在其他模块实例化) */
extern WS2812_class RGBOUT[4];

/**
 * @brief 设置指定通道的状态RGB LED颜色(直接设置，立即生效)
 * @param ch 通道号(0~3)
 * @param r  红色分量(0~255)
 * @param g  绿色分量(0~255)
 * @param b  蓝色分量(0~255)
 * @details 调用WS2812的set_RGB()直接更新指定通道的LED颜色缓冲区。
 *          仅设置颜色，不触发DMA发送(需调用RGB_update()刷新)。
 */
static inline __attribute__((always_inline))
void MC_STU_RGB_set(uint8_t ch, uint8_t r, uint8_t g, uint8_t b)
{
    if (ch < 4) RGBOUT[ch].set_RGB(r, g, b, 0);
}

/**
 * @brief 设置指定通道的在线状态RGB LED颜色(带在线/离线过渡效果)
 * @param ch       通道号(0~3)
 * @param r        红色分量(0~255)
 * @param g        绿色分量(0~255)
 * @param b        蓝色分量(0~255)
 * @param filament 是否有料盘插入(默认false)
 * @details 调用WS2812的set_RGB_online()，支持在线/离线状态的平滑过渡动画。
 *          filament参数影响LED的显示模式(如有料盘时显示实心色，无料盘时显示呼吸效果)
 */
static inline __attribute__((always_inline))
void MC_PULL_ONLINE_RGB_set(uint8_t ch, uint8_t r, uint8_t g, uint8_t b, bool filament = false)
{
    if (ch < 4) RGBOUT[ch].set_RGB_online(r, g, b, 1, filament);
}

/**
 * @brief 标记指定料盘的AMS数据需要保存
 * @param filament_idx 料盘索引号(0~3)
 * @details 通知AMS模块该料盘的数据(如类型、颜色等)已更新，需要写入Flash
 */
void ams_datas_set_need_to_save_filament(uint8_t filament_idx);

/**
 * @brief 设置指定通道的AMS状态为"已装料"
 * @param filament_ch 通道号(0~3)
 * @details 更新AMS状态机，标记该通道已装入料盘
 */
void ams_state_set_loaded(uint8_t filament_ch);

/**
 * @brief 设置指定通道的AMS状态为"已卸料"
 * @param filament_ch 通道号(0~3)
 * @details 更新AMS状态机，标记该通道的料盘已卸出
 */
void ams_state_set_unloaded(uint8_t filament_ch);

/**
 * @brief 获取当前已装料的通道号
 * @return 已装料通道号(0~3)，若无通道装料则返回特定值(如0xFF)
 * @details 查询AMS状态机，返回当前处于"已装料"状态的通道编号
 */
uint8_t ams_state_get_loaded(void);
