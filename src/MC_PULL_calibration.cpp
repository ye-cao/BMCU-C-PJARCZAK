#include "MC_PULL_calibration.h"
#include "Motion_control.h"
#include "ADC_DMA.h"
#include "Flash_saves.h"
#include "hal/time_hw.h"
#include "Debug_log.h"
#include "app_api.h"
#include "fx_units.h"

static uint8_t g_valid_mask = 0u;

bool MC_PULL_calibration_is_valid(uint8_t ch)
{
    return ch < 4u && (g_valid_mask & (1u << ch));
}

/**
 * @brief 【定点化】电压有效性检查（单位：微伏）
 * @param v_uv 电压值（微伏），对应原来的 0.05 V ~ 3.25 V
 * @return true=在有效范围内
 * @note 原 float 版为 `isfinite(v) && v >= 0.05f && v <= 3.25f`。
 *       定点化后值域是整数，不可能出现 inf/NaN，且 ADC 输出恒为 [-2^31, 2^31)，
 *       因此只需做范围比较即可。
 */
static bool finite_voltage(int32_t v_uv)
{
    return v_uv >= 50000 && v_uv <= 3250000;
}

extern void RGB_update();

/**
 * @brief 根据通道号获取拉力传感器的ADC原始电压值
 * @param ch 通道号(0~3)
 * @param v8 ADC 8通道电压数组, 单位微伏(ADC_DMA_get_value()返回)
 * @return 对应通道的拉力ADC电压值(微伏)
 * @details ADC通道映射关系: 拉力传感器与按键传感器交叉排列在8通道ADC中
 *          ch0→v8[6], ch1→v8[4], ch2→v8[2], ch3→v8[0]
 */
static inline int32_t adc_pull_raw_ch(int ch, const int32_t *v8)
{
    switch (ch)
    {
    case 0: return v8[6];
    case 1: return v8[4];
    case 2: return v8[2];
    default:return v8[0];
    }
}

/**
 * @brief 根据通道号获取按键传感器的ADC原始电压值
 * @param ch 通道号(0~3)
 * @param v8 ADC 8通道电压数组, 单位微伏
 * @return 对应通道的按键ADC电压值(微伏)
 * @details 按键传感器通道映射:
 *          ch0→v8[7], ch1→v8[5], ch2→v8[3], ch3→v8[1]
 */
static inline int32_t adc_key_raw_ch(int ch, const int32_t *v8)
{
    switch (ch)
    {
    case 0: return v8[7];
    case 1: return v8[5];
    case 2: return v8[3];
    default:return v8[1];
    }
}

/**
 * @brief 将电压值向上取整到百分位(centi-volt)并编码为uint8_t
 * @param v_uv 电压值（微伏，如 0.65 V = 650000）
 * @return 百分位编码值(如65)，范围0~255
 * @details 【定点化】原 float 版为:
 *              x = v*100 - 0.0001; iv = (int)x; if (iv < x) iv++;
 *          换成微伏后 x = (v_uv - 1) / 10000，向上取整等价于:
 *              iv = (v_uv - 1) / 10000;  if (iv*10000 < v_uv - 1) iv++;
 *          （用乘法回代判断，避免引入除法的取整方向问题）
 */
static inline uint8_t dm_key_round_up_to_centi(int32_t v_uv)
{
    if (v_uv <= 0) return 0u;

    int32_t iv = (v_uv - 1) / 10000;
    if (iv * 10000 < (v_uv - 1)) iv++;      // 向上取整

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

/**
 * @brief 根据空闲状态的按键电压值计算"无料盘"判定阈值
 * @param key_uv 空闲状态下的按键ADC平均电压值（微伏）
 * @return 无料盘判定阈值电压（微伏）
 * @details 阈值计算规则（与旧 float 版逐位一致）:
 *          1. 先将 key_uv 编码为百分位值(key_cv)
 *          2. 阈值 = key_cv + 10(百分位，即 +0.10 V)
 *          3. 最小阈值 60(0.60 V)，最大阈值 139(1.39 V)
 *          4. 由百分位换回微伏（×10000，不再走浮点）
 */
static inline int32_t dm_key_none_threshold_from_idle(int32_t key_uv)
{
    const uint8_t key_cv = dm_key_round_up_to_centi(key_uv);

    uint32_t thr_cv = (uint32_t)key_cv + 10u;  // 在空闲值基础上加0.10V
    if (thr_cv < 60u) thr_cv = 60u;    // 下限0.60V
    if (thr_cv > 139u) thr_cv = 139u;  // 上限1.39V

    return (int32_t)(thr_cv * 10000u); // 转换回微伏
}

/** @brief 【定点化】按极性翻转电压（微伏）：POLARITY<0 时返回 3.30V - v */
static inline int32_t cal_apply_polarity(int32_t v_uv, int8_t pol)
{
    return (pol < 0) ? (3300000 - v_uv) : v_uv;
}

/** @brief 按压判定阈值，0.1 V = 100000 µV */
static const int32_t  CAL_PRESS_DELTA_V_UV = 100000;
/** @brief 中心回判断阈值，0.025 V = 25000 µV */
static const int32_t  CAL_CENTER_EPS_V_UV  = 25000;
static const uint32_t CAL_STABLE_MS     = 700;
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

static bool capture_first_extreme_wait_release(int ch, int32_t center_v, int32_t offset, int32_t &out_best_norm, int8_t &out_pol)
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = (uint32_t)CAL_TIMEOUT_MS * tpm;  // 超时tick数

    bool pressed = false;       // 是否已检测到按压
    int32_t raw_min = center_v; // 按压过程中的最小电压（微伏）
    int32_t raw_max = center_v; // 按压过程中的最大电压（微伏）
    uint32_t stable_t0 = 0u;   // 电压回到中心的起始时间

    out_best_norm = center_v;
    out_pol = 1;

    for (;;)
    {
        const uint32_t now_t = time_ticks32();
        if ((uint32_t)(now_t - t0) >= dt) break;  // 超时退出

        const int32_t raw = adc_pull_raw_ch(ch, ADC_DMA_get_value());
        if (!finite_voltage(raw)) break;
        const int32_t v = raw + offset;

        // LED蓝色慢闪提示等待(每200ms交替亮/灭)
        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        if (((elapsed_ms / 200u) & 1u) == 0u) MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x10);
        else                                  MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x00);
        RGB_update();

        if (!pressed)
        {
            // 检测是否开始按压(电压偏离中心超过阈值)
            if (fx_abs(v - center_v) >= CAL_PRESS_DELTA_V_UV)
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
            if (fx_abs(v - center_v) <= CAL_CENTER_EPS_V_UV)
            {
                if (stable_t0 == 0u) stable_t0 = now_t;
                if ((uint32_t)(now_t - stable_t0) >= (uint32_t)CAL_STABLE_MS * tpm)
                {
                    // 稳定超过200ms，判定为已松手
                    const int32_t dmin = center_v - raw_min;  // 向下偏移量（微伏）
                    const int32_t dmax = raw_max - center_v;  // 向上偏移量（微伏）

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

static bool capture_second_extreme_wait_release(int ch, int32_t center_v, int32_t offset, int8_t pol, int32_t &out_best_norm)
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = (uint32_t)CAL_TIMEOUT_MS * tpm;

    const bool want_raw_min = (pol < 0);  // 极性负→需要采集更小的值

    bool pressed = false;
    int32_t best_raw = center_v; // 捕获到的极值（微伏）
    uint32_t stable_t0 = 0u;

    for (;;)
    {
        const uint32_t now_t = time_ticks32();
        if ((uint32_t)(now_t - t0) >= dt) break;

        const int32_t raw = adc_pull_raw_ch(ch, ADC_DMA_get_value());
        if (!finite_voltage(raw)) break;
        const int32_t v = raw + offset;

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
                if (v < (center_v - CAL_PRESS_DELTA_V_UV)) { pressed = true; best_raw = v; }
            }
            else
            {
                if (v > (center_v + CAL_PRESS_DELTA_V_UV)) { pressed = true; best_raw = v; }
            }
        }
        else
        {
            // 已按压：跟踪极值
            if (want_raw_min) { if (v < best_raw) best_raw = v; }
            else              { if (v > best_raw) best_raw = v; }

            // 检测松手
            if (fx_abs(v - center_v) <= CAL_CENTER_EPS_V_UV)
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

static bool capture_minmax_one_ch_event(int ch, int32_t center_v, int32_t offset, int32_t &out_min, int32_t &out_max, int8_t &out_pol)
{
    int32_t vmin = center_v;
    int32_t vmax = center_v;
    int8_t pol = 1;

    bool ok_min = capture_first_extreme_wait_release(ch, center_v, offset, vmin, pol);
    if (!ok_min) return false;
    blink_one(ch, 0x10, 0x10, 0x00, 2, 60, 60);

    bool ok_max = capture_second_extreme_wait_release(ch, center_v, offset, pol, vmax);
    if (!ok_max) return false;
    blink_one(ch, 0x10, 0x10, 0x00, 2, 60, 60);

    /* 定点化后不需要 isfinite 检查；余下三个阈值换算为微伏：0.050V=50000, 0.120V=120000 */
    if (center_v - vmin < 50000 || vmax - center_v < 50000 ||
        vmax - vmin < 120000) return false;

    out_min = vmin;
    out_max = vmax;
    out_pol = pol;
    return true;
}

bool MC_PULL_calibration_clear()
{
    g_valid_mask = 0u;
    return Flash_MC_PULL_cal_clear();
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
    g_valid_mask = 0u;
    for (int i = 0; i < 6; i++) { ADC_DMA_poll(); delay(20); }
    MC_PULL_detect_channels_inserted();
    uint8_t selected = 0u;
    for (uint8_t ch = 0u; ch < 4u; ch++)
        if (filament_channel_inserted[ch]) selected |= (uint8_t)(1u << ch);
    if (!selected) return;

    /* 【定点化】电压相关量全部改为 int32 微伏；中心点 1.65V = 1650000 */
    int32_t offs[4] = {}, vmin[4] = {}, vmax[4] = {}, dm_none[4] = {};
    int8_t pol[4] = {1, 1, 1, 1};
    uint8_t valid = 0u;
    bool stored = Flash_MC_PULL_cal_read(offs, vmin, vmax, pol, dm_none, &valid);
    if (stored)
    {
        for (uint8_t ch = 0u; ch < 4u; ch++)
        {
            if (!(valid & (1u << ch))) continue;
            /* 原来还有 isfinite() 检查；定点化后不存在 inf/NaN，去掉即可。
             * 阈值换算：0.050V=50000µV, 0.120V=120000µV, 0.60V=600000µV, 1.39V=1390000µV */
            if (!finite_voltage(1650000 - offs[ch]) ||
                1650000 - vmin[ch] < 50000 || vmax[ch] - 1650000 < 50000 ||
                vmax[ch] - vmin[ch] < 120000 ||
                dm_none[ch] < 600000 || dm_none[ch] > 1390000)
                valid &= (uint8_t)~(1u << ch);
        }
        stored = (valid & selected) == selected;
    }

    if (!stored)
    {
        /* 60 次采样、每次最大 3,250,000 µV ⇒ 累加最大 1.95e8，int32 足够 */
        int32_t sum_raw[4] = {}, sum_key[4] = {};
        int32_t low[4] = {10000000, 10000000, 10000000, 10000000}, high[4] = {};
        /* 注：low 的初值 10 原意是 10 V（远超量程，仅用于 high-low 的稳定性判据） */
        uint8_t baseline_valid = selected;
        const int samples = 60;
        for (int k = 0; k < samples; k++)
        {
            const int32_t *v = ADC_DMA_get_value();
            for (uint8_t ch = 0u; ch < 4u; ch++)
            {
                if (!(selected & (1u << ch))) continue;
                const int32_t raw = adc_pull_raw_ch(ch, v);
                const int32_t key = adc_key_raw_ch(ch, v);
                if (!finite_voltage(raw) || key < 0 || key >= 1400000)
                    baseline_valid &= (uint8_t)~(1u << ch);
                sum_raw[ch] += raw;
                sum_key[ch] += key;
                if (raw < low[ch]) low[ch] = raw;
                if (raw > high[ch]) high[ch] = raw;
                const bool on = ((k / 15) & 1) == 0;
                MC_PULL_ONLINE_RGB_set(ch, on ? 0x0Cu : 0u, on ? 0x0Cu : 0u, 0u);
            }
            RGB_update();
            delay(15);
        }
        for (uint8_t ch = 0u; ch < 4u; ch++)
        {
            offs[ch]    = 0;          /* 0 V */
            vmin[ch]    = 1500000;    /* 1.50 V */
            vmax[ch]    = 1800000;    /* 1.80 V */
            dm_none[ch] = 600000;     /* 0.60 V */
            pol[ch] = 1;
            if (!(selected & (1u << ch))) continue;
            if (high[ch] - low[ch] > 40000) baseline_valid &= (uint8_t)~(1u << ch);
            if (!(baseline_valid & (1u << ch))) continue;
            offs[ch]    = 1650000 - sum_raw[ch] / samples;   /* samples=60 常量除法 */
            dm_none[ch] = dm_key_none_threshold_from_idle(sum_key[ch] / samples);
            MC_PULL_ONLINE_RGB_set(ch, 0u, 0u, 0u);
        }
        RGB_update();
        if (baseline_valid != selected) { blink_all(0x18u, 0u, 0u, 2, 220, 220); return; }

        valid = 0u;
        for (uint8_t ch = 0u; ch < 4u; ch++)
        {
            if (!(selected & (1u << ch))) continue;
            if (!capture_minmax_one_ch_event(ch, 1650000, offs[ch], vmin[ch], vmax[ch], pol[ch]))
            {
                blink_all(0x18u, 0u, 0u, 2, 220, 220);
                return;
            }
            valid |= (uint8_t)(1u << ch);
            MC_PULL_ONLINE_RGB_set(ch, 0x10u, 0x10u, 0u);
            RGB_update();
        }

        const uint32_t started = time_ticks32();
        uint32_t centered_since = 0u;
        bool released = false;
        while ((uint32_t)(time_ticks32() - started) < ms_to_ticks32(CAL_TIMEOUT_MS))
        {
            const int32_t *v = ADC_DMA_get_value();
            bool centered = true;
            for (uint8_t ch = 0u; ch < 4u; ch++)
            {
                if (!(selected & (1u << ch))) continue;
                const int32_t raw = adc_pull_raw_ch(ch, v);
                const int32_t key = adc_key_raw_ch(ch, v);
                /* 0.080 V = 80000 µV */
                if (!finite_voltage(raw) || key < 0 || key >= dm_none[ch] ||
                    fx_abs(raw + offs[ch] - 1650000) > 80000) centered = false;
            }
            const uint32_t now = time_ticks32();
            if (!centered) centered_since = 0u;
            else if (!centered_since) centered_since = now;
            else if ((uint32_t)(now - centered_since) >= ms_to_ticks32(1200u))
            {
                released = true;
                break;
            }
            delay(15);
        }
        if (!released || valid != selected ||
            !Flash_MC_PULL_cal_write_all(offs, vmin, vmax, pol, dm_none, valid))
        {
            blink_all(0x18u, 0u, 0u, 2, 220, 220);
            return;
        }
    }

    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (!(valid & (1u << ch))) continue;
        MC_PULL_V_OFFSET[ch] = offs[ch];
        MC_PULL_V_MIN[ch] = vmin[ch];
        MC_PULL_V_MAX[ch] = vmax[ch];
        MC_PULL_POLARITY[ch] = pol[ch];
        MC_DM_KEY_NONE_THRESH[ch] = dm_none[ch];
    }
    g_valid_mask = valid & selected;
    if (!stored) blink_all(0u, 0x16u, 0u, 2, 220, 220);
}
