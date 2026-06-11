#pragma once

/**
 * @file MC_PULL_calibration.h
 * @brief 料盘拉力电机校准模块头文件
 * @details 用于BMCU-C智能料盘架的拉力检测ADC校准。
 *          通过采集各通道的ADC基准电压和极值，建立电压-拉力映射关系，
 *          并将校准参数保存到Flash中，供运行时读取使用。
 */

/**
 * @brief 开机执行拉力校准流程
 * @details 校准流程:
 *          1. 尝试从Flash读取已有校准数据，若有则直接使用
 *          2. 若无校准数据，执行完整校准:
 *             a. 采集空闲状态下的ADC均值作为中心电压
 *             b. 逐通道引导用户按压料盘，采集最大/最小极值
 *             c. 将校准参数(偏移量、极值、极性)写入Flash
 *          3. 通过RGB LED闪烁指示校准进度和结果
 */
void MC_PULL_calibration_boot();

/**
 * @brief 清除所有校准数据(NVM全擦除)
 * @details 调用Flash_NVM_full_clear()擦除整个NVM存储区，
 *          下次开机将重新执行完整校准流程
 */
void MC_PULL_calibration_clear();
