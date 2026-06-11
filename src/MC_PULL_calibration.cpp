#include "MC_PULL_calibration.h"
#include "Motion_control.h"
#include "ADC_DMA.h"
#include "Flash_saves.h"
#include "hal/time_hw.h"
#include "Debug_log.h"
#include "app_api.h"
#include <math.h>

/** @brief 外部声明，RGB LED刷新函数(在其他模块实现) */
extern void RGB_update();

/**
 * @brief 根据通道号获取拉力传感器的ADC原始电压值
 * @param ch 通道号(0~3)
 * @param v8 ADC 8通道浮点电压数组(ADC_DMA_get_value()返回)
 * @return 对应通道的拉力ADC电压值
 * @details ADC通道映射关系: 拉力传感器与按键传感器交叉排列在8通道ADC中
 *          ch0→v8[6], ch1→v8[4], ch2→v8[2], ch3→v8[0]
 */
static inline float adc_pull_raw_ch(int ch, const float *v8)
{
    switch (ch)
    {
    case 0: return (float)v8[6];
    case 1: return (float)v8[4];
    case 2: return (float)v8[2];
    default:return (float)v8[0];
    }
}

/**
 * @brief 根据通道号获取按键传感器的ADC原始电压值
 * @param ch 通道号(0~3)
 * @param v8 ADC 8通道浮点电压数组
 * @return 对应通道的按键ADC电压值
 * @details 按键传感器通道映射:
 *          ch0→v8[7], ch1→v8[5], ch2→v8[3], ch3→v8[1]
 */
static inline float adc_key_raw_ch(int ch, const float *v8)
{
    switch (ch)
    {
    case 0: return (float)v8[7];
    case 1: return (float)v8[5];
    case 2: return (float)v8[3];
    default:return (float)v8[1];
    }
}

/**
 * @brief 将浮点电压值四舍五入到百分位(centi-volt)并编码为uint8_t
 * @param v 电压值(如0.65V)
 * @return 百分位编码值(如65)，范围0~255
 * @details 用于按键阈值的存储和计算:
 *          1. 若v<=0返回0
 *          2. v*100 - 0.0001(避免浮点精度问题)
 *          3. 截断取整后+1(实现向上取整)
 *          4. 限制在0~255范围内
 */
static inline uint8_t dm_key_round_up_to_centi(float v)
{
    if (v <= 0.0f) return 0u;

    float x = v * 100.0f - 0.0001f;
    int iv = (int)x;
    if ((float)iv < x) iv++;    // 向上取整

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

/**
 * @brief 根据空闲状态的按键电压值计算"无料盘"判定阈值
 * @param key_value 空闲状态下的按键ADC平均电压值
 * @return 无料盘判定阈值电压(V)
 * @details 阈值计算规则:
 *          1. 先将key_value编码为百分位值(key_cv)
 *          2. 阈值 = key_cv + 10(百分位，即+0.10V)
 *          3. 最小阈值60(0.60V)，最大阈值139(1.39V)
 *          这个阈值用于判断料盘是否已插入(超过阈值=有料盘)
 */
static inline float dm_key_none_threshold_from_idle(float key_value)
{
    const uint8_t key_cv = dm_key_round_up_to_centi(key_value);

    uint8_t thr_cv = (uint8_t)(key_cv + 10u);  // 在空闲值基础上加0.10V
    if (thr_cv < 60u) thr_cv = 60u;    // 下限0.60V
    if (thr_cv > 139u) thr_cv = 139u;  // 上限1.39V

    return 0.01f * (float)thr_cv;      // 转换回浮点电压值
}

/**
 * @brief 获取指定通道的中心校准电压值(已加上偏移量)
 * @param ch 通道号(0~3)
 * @return 校准后的中心电压值
 * @details 读取ADC原始电压后加上MC_PULL_V_OFFSET[ch]偏移量，
 *          使得理想空闲状态下的电压中心为1.65V(参考电压)
 */
static inline float adc_pull_v_centered(int ch)
{
    const float *d = ADC_DMA_get_value();
    return adc_pull_raw_ch(ch, d) + MC_PULL_V_OFFSET[ch];
}

/**
 * @brief 根据极性标志应用电压极性翻转
 * @param v  原始电压值
 * @param pol 极性标志(1=正向，-1=反向)
 * @return 极性校正后的电压值
 * @details 若极性为负，返回3.30V - v(ADC满量程3.3V，反向翻转)
 *          用于适配不同方向安装的电位器/传感器
 */
static inline float cal_apply_polarity(float v, int8_t pol)
{
    return (pol < 0) ? (3.30f - v) : v;
}

// --- 校准参数常量 ---
/** @brief 触发"已按压"判定的最小电压变化量(0.10V) */
static const float CAL_PRESS_DELTA_V = 0.1f;

/** @brief 判定"已回到中心"的最大电压偏差(0.02V) */
static const float CAL_CENTER_EPS_V  = 0.02f;

/** @brief 判定电压稳定的最短持续时间(200ms) */
static const uint32_t CAL_STABLE_MS     = 200;

/** @brief 等待用户操作的最大超时时间(30秒) */
static const uint32_t CAL_TIMEOUT_MS    = 30000;

/**
 * @brief 让所有4个通道的RGB LED同时闪烁指定颜色
 * @param r,g,b  RGB颜色分量(0x00~0x10, 低亮度)
 * @param times  闪烁次数(默认4次)
 * @param on_ms  亮灯持续时间(默认60ms)
 * @param off_ms 灭灯持续时间(默认60ms)
 * @details 用于校准过程中的视觉反馈提示
 */
static void blink_all(uint8_t r, uint8_t g, uint8_t b, int times = 4, int on_ms = 60, int off_ms = 60)
{
    for (int k = 0; k < times; k++)
    {
        for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, r, g, b);
        RGB_update(); delay(on_ms);
        for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update(); delay(off_ms);
    }
}

/**
 * @brief 让指定单个通道的RGB LED闪烁指定颜色
 * @param ch     通道号(0~3)
 * @param r,g,b  RGB颜色分量
 * @param times  闪烁次数(默认3次)
 * @param on_ms  亮灯持续时间(默认70ms)
 * @param off_ms 灭灯持续时间(默认70ms)
 * @details 用于单通道校准完成后的反馈提示
 */
static void blink_one(int ch, uint8_t r, uint8_t g, uint8_t b, int times = 3, int on_ms = 70, int off_ms = 70)
{
    for (int k = 0; k < times; k++)
    {
        MC_PULL_ONLINE_RGB_set(ch, r, g, b);
        RGB_update(); delay(on_ms);
        MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update(); delay(off_ms);
    }
}

/**
 * @brief 显示一个诊断步骤的RGB闪烁(所有通道同色)
 * @param r,g,b  RGB颜色分量
 * @param on_ms  亮灯持续时间(默认360ms)
 * @param off_ms 灭灯持续时间(默认180ms)
 * @details 用于校准结果的分步指示:
 *          短闪烁=步骤成功，长闪烁=步骤失败
 */
static void show_diag_step(uint8_t r, uint8_t g, uint8_t b, int on_ms = 360, int off_ms = 180)
{
    for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, r, g, b);
    RGB_update();
    delay(on_ms);
    for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
    delay(off_ms);
}

/**
 * @brief 捕获第一个极值(首次按压方向)并等待用户松手
 * @param ch             通道号(0~3)
 * @param center_v       中心校准电压值
 * @param out_best_norm  [输出] 捕获到的极值(极性校正后)
 * @param out_pol        [输出] 检测到的极性(1=正向，-1=反向)
 * @return true=成功捕获并等待松手，false=超时未完成
 * @details 校准流程第一步:
 *          1. 等待用户按压(电压偏离中心超过CAL_PRESS_DELTA_V)
 *          2. 记录按压过程中的最大/最小电压
 *          3. 等待用户松手(电压回到中心附近持续200ms)
 *          4. 比较最大/最小偏差确定极性(偏差大的方向=主要按压方向)
 *          5. 返回极性校正后的极值和极性标志
 *          超时30秒，期间LED以蓝色慢闪提示等待
 */
static bool capture_first_extreme_wait_release(int ch, float center_v, float &out_best_norm, int8_t &out_pol)
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = (uint32_t)CAL_TIMEOUT_MS * tpm;  // 超时tick数

    bool pressed = false;       // 是否已检测到按压
    float raw_min = center_v;   // 按压过程中的最小电压
    float raw_max = center_v;   // 按压过程中的最大电压
    uint32_t stable_t0 = 0u;   // 电压回到中心的起始时间

    out_best_norm = center_v;
    out_pol = 1;

    for (;;)
    {
        const uint32_t now_t = time_ticks32();
        if ((uint32_t)(now_t - t0) >= dt) break;  // 超时退出

        const float v = adc_pull_v_centered(ch);  // 读取中心校准电压

        // LED蓝色慢闪提示等待(每200ms交替亮/灭)
        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        if (((elapsed_ms / 200u) & 1u) == 0u) MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x10);
        else                                  MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x00);
        RGB_update();

        if (!pressed)
        {
            // 检测是否开始按压(电压偏离中心超过阈值)
            if (fabsf(v - center_v) >= CAL_PRESS_DELTA_V)
            {
                pressed = true;
                raw_min = v;
                raw_max = v;
            }
        }
        else
        {
            // 已按压：记录极值
            if (v < raw_min) raw_min = v;
            if (v > raw_max) raw_max = v;

            // 检测是否松手(电压回到中心附近)
            if (fabsf(v - center_v) <= CAL_CENTER_EPS_V)
            {
                if (stable_t0 == 0u) stable_t0 = now_t;
                if ((uint32_t)(now_t - stable_t0) >= (uint32_t)CAL_STABLE_MS * tpm)
                {
                    // 稳定超过200ms，判定为已松手
                    const float dmin = center_v - raw_min;  // 向下偏移量
                    const float dmax = raw_max - center_v;  // 向上偏移量

                    if (dmax > dmin)
                    {
                        out_pol = -1;  // 主要偏移方向为正 → 极性反向
                        out_best_norm = cal_apply_polarity(raw_max, out_pol);
                    }
                    else
                    {
                        out_pol = 1;   // 主要偏移方向为负 → 极性正向
                        out_best_norm = cal_apply_polarity(raw_min, out_pol);
                    }

                    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
                    RGB_update();
                    return true;
                }
            }
            else
            {
                stable_t0 = 0u;  // 电压未回到中心，重置稳定计时
            }
        }

        delay(15);  // 15ms采样间隔
    }

    // 超时，返回默认值
    out_best_norm = center_v;
    out_pol = 1;
    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
    return false;
}

/**
 * @brief 捕获第二个极值(与第一个极值相反的方向)并等待松手
 * @param ch             通道号(0~3)
 * @param center_v       中心校准电压值
 * @param pol            第一步检测到的极性
 * @param out_best_norm  [输出] 捕获到的极值(极性校正后)
 * @return true=成功捕获，false=超时未完成
 * @details 校准流程第二步:
 *          1. 根据极性决定等待哪个方向的按压(pol<0→等待raw_min，pol>0→等待raw_max)
 *          2. 捕获该方向的极值并等待松手
 *          3. 返回极性校正后的极值
 *          LED以红色慢闪提示等待
 */
static bool capture_second_extreme_wait_release(int ch, float center_v, int8_t pol, float &out_best_norm)
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = (uint32_t)CAL_TIMEOUT_MS * tpm;

    const bool want_raw_min = (pol < 0);  // 极性负→需要采集更小的值

    bool pressed = false;
    float best_raw = center_v;   // 捕获到的极值
    uint32_t stable_t0 = 0u;

    for (;;)
    {
        const uint32_t now_t = time_ticks32();
        if ((uint32_t)(now_t - t0) >= dt) break;

        const float v = adc_pull_v_centered(ch);

        // LED红色慢闪提示等待
        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        if (((elapsed_ms / 200u) & 1u) == 0u) MC_PULL_ONLINE_RGB_set(ch, 0x10, 0x00, 0x00);
        else                                  MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x00);
        RGB_update();

        if (!pressed)
        {
            // 根据极性检测对应方向的按压
            if (want_raw_min)
            {
                if (v < (center_v - CAL_PRESS_DELTA_V)) { pressed = true; best_raw = v; }
            }
            else
            {
                if (v > (center_v + CAL_PRESS_DELTA_V)) { pressed = true; best_raw = v; }
            }
        }
        else
        {
            // 已按压：跟踪极值
            if (want_raw_min) { if (v < best_raw) best_raw = v; }
            else              { if (v > best_raw) best_raw = v; }

            // 检测松手
            if (fabsf(v - center_v) <= CAL_CENTER_EPS_V)
            {
                if (stable_t0 == 0u) stable_t0 = now_t;
                if ((uint32_t)(now_t - stable_t0) >= (uint32_t)CAL_STABLE_MS * tpm)
                {
                    out_best_norm = cal_apply_polarity(best_raw, pol);
                    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
                    RGB_update();
                    return true;
                }
            }
            else
            {
                stable_t0 = 0u;
            }
        }

        delay(15);
    }

    out_best_norm = center_v;
    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
    return false;
}

/**
 * @brief 捕获单个通道的最小/最大极值和极性(交互式校准)
 * @param ch        通道号(0~3)
 * @param center_v  中心校准电压值
 * @param out_min   [输出] 校准后的最小电压(极性已应用)
 * @param out_max   [输出] 校准后的最大电压(极性已应用)
 * @param out_pol   [输出] 检测到的极性
 * @details 完整的单通道校准流程:
 *          1. capture_first_extreme_wait_release() - 捕获第一个极值
 *          2. capture_second_extreme_wait_release() - 捕获第二个极值
 *          3. 确保最小值与中心值偏差>=50mV，最大值与中心值偏差>=50mV
 *          4. 确保最小值和最大值之间至少有100mV的间距
 *          5. 校准完成后闪烁黄色LED提示
 */
static void capture_minmax_one_ch_event(int ch, float center_v, float &out_min, float &out_max, int8_t &out_pol)
{
    float vmin = center_v;
    float vmax = center_v;
    int8_t pol = 1;

    bool ok_min = capture_first_extreme_wait_release(ch, center_v, vmin, pol);
    if (ok_min) blink_one(ch, 0x10, 0x10, 0x00, 2, 60, 60);  // 黄色闪烁=第一步成功

    bool ok_max = capture_second_extreme_wait_release(ch, center_v, pol, vmax);
    if (ok_max) blink_one(ch, 0x10, 0x10, 0x00, 2, 60, 60);  // 黄色闪烁=第二步成功

    // 确保最小值与中心值至少有50mV的间距
    if (vmin > (center_v - 0.050f)) vmin = (center_v - 0.050f);
    // 确保最大值与中心值至少有50mV的间距
    if (vmax < (center_v + 0.050f)) vmax = (center_v + 0.050f);
    // 确保最小值和最大值之间至少有100mV的间距
    if (vmax <= vmin + 0.10f) { vmin = center_v - 0.10f; vmax = center_v + 0.10f; }

    out_min = vmin;
    out_max = vmax;
    out_pol = pol;
}

/**
 * @brief 清除所有校准数据(擦除整个NVM存储区)
 * @details 调用Flash_NVM_full_clear()擦除Flash中保存的所有校准参数，
 *          下次开机将重新执行完整校准流程
 */
void MC_PULL_calibration_clear()
{
    Flash_NVM_full_clear();
}

/**
 * @brief 开机拉力校准主流程
 * @details 完整校准流程:
 *          1. 预采样: ADC采集6次(每次20ms)稳定传感器
 *          2. 检测料盘是否已插入
 *          3. 尝试从Flash读取已有校准数据:
 *             - 若有数据 → 直接加载到全局变量，返回
 *             - 若无数据 → 执行完整校准
 *          4. 完整校准:
 *             a. 清除旧校准数据
 *             b. 采集90次ADC均值作为基准(同时LED黄色慢闪)
 *             c. 计算各通道中心电压偏移量(目标中心1.65V)
 *             d. 计算按键无料盘判定阈值
 *             e. 逐通道交互式校准(引导用户按压)
 *             f. 将校准参数写入Flash
 *             g. LED闪烁指示每步结果(青色=擦除成功，黄色=校准成功，蓝色=按键保存成功)
 *             h. 最终LED: 绿色=全部成功，红色=有失败
 */
void MC_PULL_calibration_boot()
{
    // 预采样: 采集6次稳定ADC
    for (int i = 0; i < 6; i++) { ADC_DMA_poll(); delay(20); }

    // 检测各通道料盘是否已插入
    MC_PULL_detect_channels_inserted();

    // 尝试读取已有校准数据
    float offs[4], vmin[4], vmax[4];
    int8_t pol[4];
    if (Flash_MC_PULL_cal_read(offs, vmin, vmax, pol))
    {
        // 读取成功，直接加载校准参数
        for (int ch = 0; ch < 4; ch++)
        {
            MC_PULL_V_OFFSET[ch] = offs[ch];      /* 中心电压偏移量 */
            MC_PULL_V_MIN[ch]    = vmin[ch];       /* 校准最小电压 */
            MC_PULL_V_MAX[ch]    = vmax[ch];       /* 校准最大电压 */
            MC_PULL_POLARITY[ch] = (pol[ch] < 0) ? -1 : 1;  /* 极性标志 */
        }
        return;  // 已有校准数据，跳过校准
    }

    // 无校准数据，执行完整校准
    const bool ok_wipe = Flash_NVM_full_clear();  // 清除旧数据

    // --- 第一阶段: 采集空闲状态ADC均值 ---
    double sum_raw[4] = {0, 0, 0, 0};   // 拉力传感器累加和
    double sum_key[4] = {0, 0, 0, 0};   // 按键传感器累加和
    const int N = 90;                     // 采样次数
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();

    for (int k = 0; k < N; k++)
    {
        const float *v = ADC_DMA_get_value();

        for (int ch = 0; ch < 4; ch++)
        {
            if (!filament_channel_inserted[ch]) continue;  // 未插入的通道跳过
            sum_raw[ch] += adc_pull_raw_ch(ch, v);
            sum_key[ch] += adc_key_raw_ch(ch, v);
        }

        // LED黄色慢闪指示采样进行中
        const uint32_t now_t = time_ticks32();
        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        bool on = (((elapsed_ms / 200u) & 1u) == 0u);
        for (int ch = 0; ch < 4; ch++)
            MC_PULL_ONLINE_RGB_set(ch, on ? 0x10 : 0, on ? 0x10 : 0, 0x00);
        RGB_update();
        delay(15);
    }

    // 计算各通道中心电压(默认1.65V，未插入通道保持默认)
    float center_raw[4] = {1.65f, 1.65f, 1.65f, 1.65f};
    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch]) continue;
        center_raw[ch] = (float)(sum_raw[ch] / (double)N);  // 90次采样均值
    }

    // --- 第二阶段: 计算偏移量和按键阈值 ---
    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch])
        {
            // 未插入通道使用默认值
            MC_PULL_V_OFFSET[ch] = 0.0f;
            MC_PULL_POLARITY[ch] = 1;
            MC_DM_KEY_NONE_THRESH[ch] = 0.60f;
            continue;
        }

        // 偏移量 = 1.65V - 实测中心值(使校准后中心归一化到1.65V)
        MC_PULL_V_OFFSET[ch] = 1.65f - center_raw[ch];
        MC_PULL_POLARITY[ch] = 1;
        // 按键无料盘阈值(基于空闲按键电压计算)
        MC_DM_KEY_NONE_THRESH[ch] =
            dm_key_none_threshold_from_idle((float)(sum_key[ch] / (double)N));
    }

    // 计算校准后的中心电压参考值(加上偏移量后应接近1.65V)
    float center_v_ref[4] = {1.65f, 1.65f, 1.65f, 1.65f};
    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch]) continue;
        center_v_ref[ch] = center_raw[ch] + MC_PULL_V_OFFSET[ch];
    }

    // 黄色闪烁提示进入交互式校准阶段
    blink_all(0x10, 0x10, 0x00, 3, 60, 60);

    // --- 第三阶段: 逐通道交互式校准(引导用户按压) ---
    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch])
        {
            // 未插入通道使用默认极值范围
            MC_PULL_V_MIN[ch] = 1.55f;
            MC_PULL_V_MAX[ch] = 1.75f;
            MC_PULL_POLARITY[ch] = 1;
            continue;
        }

        float mn, mx;
        int8_t p;
        capture_minmax_one_ch_event(ch, center_v_ref[ch], mn, mx, p);  // 交互式校准

        MC_PULL_V_MIN[ch] = mn;
        MC_PULL_V_MAX[ch] = mx;
        MC_PULL_POLARITY[ch] = (p < 0) ? -1 : 1;

        // 关闭该通道LED，短暂延时后继续下一通道
        MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update();
        delay(80);
    }

    // --- 第四阶段: 保存校准参数到Flash ---
    const bool ok_cal = Flash_MC_PULL_cal_write_all(MC_PULL_V_OFFSET, MC_PULL_V_MIN, MC_PULL_V_MAX, MC_PULL_POLARITY);
    const bool ok_mot = Motion_control_save_dm_key_none_thresholds();
    const bool ok = ok_wipe && ok_cal && ok_mot;

    // LED分步显示保存结果
    if (ok_wipe) show_diag_step(0x00, 0x10, 0x10);   // 青色=擦除成功
    else         show_diag_step(0x10, 0x00, 0x00, 460, 220);  // 红色=擦除失败

    if (ok_cal) show_diag_step(0x10, 0x10, 0x00);   // 黄色=校准参数保存成功
    else        show_diag_step(0x10, 0x00, 0x10, 460, 220);  // 紫色=校准参数保存失败

    if (ok_mot) show_diag_step(0x00, 0x00, 0x10);   // 蓝色=按键阈值保存成功
    else        show_diag_step(0x10, 0x10, 0x10, 460, 220);  // 白色=按键阈值保存失败

    // 最终结果: 绿色=全部成功，红色=有失败
    if (ok) blink_all(0x00, 0x10, 0x00, 2, 220, 220);
    else    blink_all(0x10, 0x00, 0x00, 2, 260, 260);

    delay(200);  // 结束后短暂延时
}
