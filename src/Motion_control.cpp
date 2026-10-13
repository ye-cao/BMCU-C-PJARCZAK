/**
 * @file    Motion_control.cpp
 * @brief   BMCU-C 运动控制模块实现 —— 4通道耗材架完整运动逻辑
 *
 * 主要功能模块：
 *   1. AS5600 磁性编码器数据读取与速度计算
 *   2. PID 电机速度/压力控制器
 *   3. 电机 PWM 输出（4通道，通过 TIM2/TIM3/TIM4）
 *   4. ADC 传感器读取（速度传感器 + 微动开关）
 *   5. 运动状态机（idle/send/pull/on_use/pull_back 等）
 *   6. 双微动开关自动加载/卸料辅助（DM 型号）
 *   7. 堵转检测与保护、校准重置、电机方向校准
 */
#include "Motion_control.h"
#include "MC_PULL_calibration.h"
#include "ams.h"
#include "ADC_DMA.h"
#include "Flash_saves.h"
#include "_bus_hardware.h"
#include "many_soft_AS5600.h"
#include "app_api.h"
#include "hal/time_hw.h"
#include "tpu_params.h"
#include "fx_units.h"

/* ==================== 定点化单位约定 ====================
 * 详细约定与溢出铁律见 fx_units.h。本文件统一使用：
 *   PWM / 推力    int32_t  毫 PWM (0.001)   满量程 1000 = 1000000，范围 ±1000000
 *   压力百分比    int32_t  厘百分比 (0.01%) 50 % = 5000，范围 0 ~ 10000
 *   电压          int32_t  微伏 (µV)        1.65 V = 1650000
 *   电机速度      int32_t  微米/秒 (µm/s)   60 mm/s = 60000
 *   位移 / 距离   int32_t  微米 (µm)        0.120 m = 120000
 *   时间步长      int32_t  微秒 (µs)
 * 任何定点乘法的中间结果都必须落在 int32 内；一旦引 入 int64 乘法/除法，
 * 就会把 __muldi3 / __divdi3 / __udivdi3 拉回来（各数百字节），得不偿失。
 * ====================================================== */

/** @brief 毫 PWM 倍数：1 个 PWM 单位（= 0.1 % 占空比）= 1000 毫 PWM */
static constexpr int32_t kPwmM = 1000;
/** @brief 厘百分比倍数：1 % = 100 厘百分比 */
static constexpr int32_t kPctC = 100;
/** @brief 千分之一秒：时间步长换算用（time_us / kUsPerMs = 毫秒） */
static constexpr int32_t kUsPerMs = 1000;
/** @brief 百万分之一米：微米 → 米 的换算分母 */
static constexpr int32_t kUmPerM = 1000000;

/**
 * @brief 将电压值转换为 centi-volt 格式（向上取整），用于存储到 uint8_t
 *        centi-volt = voltage * 100，向上取整确保不低估阈值
 * @param v_uv 电压（微伏）
 * @note 【定点化】原 float 版为 x = v*100 - 0.0001 后向上取整。
 *       换成微伏后 x = (v_uv - 1)/10000，向上取整等价于
 *       iv = (v_uv-1)/10000 且若 iv*10000 < v_uv-1 则 iv++（乘法回代，避免取整方向歧义）
 */
static inline uint8_t dm_key_v_to_centi_ceil(int32_t v_uv)
{
    if (v_uv <= 0) return 0u;

    int32_t iv = (v_uv - 1) / 10000;
    if (iv * 10000 < (v_uv - 1)) iv++;   // 向上取整

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

/* ==================== 高精度时间系统 ==================== */

/** @brief 上次读取的 64 位硬件 tick 值 */
static uint64_t g_time_last_ticks64 = 0ull;
/** @brief 除法后剩余的 tick 数（用于累积避免精度损失） */
static uint32_t g_time_rem_ticks32  = 0u;
/** @brief 累积的毫秒数 */
static uint64_t g_time_ms64         = 0ull;
/** @brief 上次使用的 ticks-per-ms 值（检测时钟变化） */
static uint32_t g_time_tpm_last     = 0u;
/** @brief 时间系统是否已初始化标志 */
static uint8_t  g_time_inited       = 0u;

/**
 * @brief 从 64 位硬件 tick 快速转换为毫秒（增量累加方式）
 *        避免每次除法运算，仅在时钟变化或溢出时完整除法
 */
static inline __attribute__((always_inline)) uint64_t time_ms_fast_from_ticks64(uint64_t now_ticks)
{
    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    if (!g_time_inited || (tpm != g_time_tpm_last))
    {
        g_time_inited = 1u;
        g_time_tpm_last = tpm;
        g_time_last_ticks64 = now_ticks;

        g_time_ms64 = now_ticks / (uint64_t)tpm;
        g_time_rem_ticks32 = (uint32_t)(now_ticks - g_time_ms64 * (uint64_t)tpm);
        return g_time_ms64;
    }

    const uint64_t dt64 = now_ticks - g_time_last_ticks64;
    g_time_last_ticks64 = now_ticks;

    if (__builtin_expect(dt64 > 0xFFFFFFFFull, 0))
    {
        g_time_ms64 = now_ticks / (uint64_t)tpm;
        g_time_rem_ticks32 = (uint32_t)(now_ticks - g_time_ms64 * (uint64_t)tpm);
        return g_time_ms64;
    }

    const uint32_t dt  = (uint32_t)dt64;
    const uint32_t rem = g_time_rem_ticks32;

    if (__builtin_expect(dt > (0xFFFFFFFFu - rem), 0))
    {
        g_time_ms64 = now_ticks / (uint64_t)tpm;
        g_time_rem_ticks32 = (uint32_t)(now_ticks - g_time_ms64 * (uint64_t)tpm);
        return g_time_ms64;
    }

    const uint32_t acc = dt + rem;

    if (tpm <= 1u)
    {
        g_time_ms64 += (uint64_t)acc;
        g_time_rem_ticks32 = 0u;
        return g_time_ms64;
    }

    const uint32_t inc = acc / tpm;
    g_time_rem_ticks32 = acc - inc * tpm;

    g_time_ms64 += (uint64_t)inc;
    return g_time_ms64;
}

static inline __attribute__((always_inline)) uint64_t time_ms_fast(void)
{
    return time_ms_fast_from_ticks64(time_ticks64());
}

/**
 * @brief 根据压力误差计算回退力矩（分段线性映射）
 *        err<=10:无回退, 10~35:450000~550000, 35~235:550000~850000
 * @param err     压力误差，单位厘百分比（0.01%）
 * @param mag_max 力矩上限，单位毫 PWM（0.001）
 * @return 回退力矩，单位毫 PWM
 * @note 【定点化】分段插值参数 t 用 Q10 定点表示（1024 = 1.0），
 *       避免浮点除法；最大中间积 300000×1024 = 3.07e8 < 2^31 ✓
 */
static inline int32_t retract_mag_from_err(int32_t err, int32_t mag_max)
{
    /* 分段点（单位：厘百分比）：0.10 % = 10，0.35 % = 35，2.35 % = 235 */
    constexpr int32_t e0 = 10;
    constexpr int32_t e1 = 35;
    constexpr int32_t e2 = 235;

    if (err <= e0) return 0;

    int32_t mag;
    if (err < e1)
    {
        int32_t t = ((err - e0) * 1024) / (e1 - e0);
        t = fx_clamp(t, 0, 1024);
        mag = 450 * kPwmM + (100 * kPwmM * t) / 1024;
    }
    else
    {
        int32_t t = ((err - e1) * 1024) / (e2 - e1);
        t = fx_clamp(t, 0, 1024);
        mag = 550 * kPwmM + (300 * kPwmM * t) / 1024;
    }

    if (mag > mag_max) mag = mag_max;
    return mag;
}


/** @brief 滞回比较器：防止阈值边界频繁切换（start=激活, stop=停止）
 *  【定点化】v/start/stop 统一为 int32 定点量（厘百分比） */
static inline uint8_t hyst_u8(uint8_t active, int32_t v, int32_t start, int32_t stop)
{
    if (active) { if (v <= stop)  active = 0; }
    else        { if (v >= start) active = 1; }
    return active;
}


/** @brief 通道数量（固定4通道） */
static constexpr uint8_t  kChCount = 4;
/** @brief PWM 最大值（1000 = 100% 占空比） */
static constexpr int      PWM_lim  = 1000;
/** @brief AS5600 每计数对应的耗材位移绝对值，单位：纳米（nm/count）
 *  推导：滚轮周长 π×7.5mm = 23561.9449 µm/圈，每圈 4096 计数
 *        ⇒ 23561.9449 / 4096 = 5.752428 µm/count = 5752.428 nm/count
 *  【定点化】原为 `-(π*7.5f)/4096.0f`（mm/count）的 float 常量。
 *            改为整数常量后，运行期与常量池里都不再出现浮点。
 *  约定：AP 计数增加 = 耗材回退（对应原实现里负系数 kAS5600_MM_PER_CNT 的语义）。 */
static constexpr int32_t kAS5600_NM_PER_CNT = 5752;   /* 5.752428 µm/count 截断到 nm */

/** @brief 速度换算系数：µm/count × 1e6 = 5752428（负号 = 计数增加即回退）
 *  速度(µm/s) = 计数差 × kAS5600_SPEED_NUM / 时间(µs)
 *  取值与上游 float 实现逐位对齐：dist_mm × inv_dt
 *  = diff × (-5.752428e-3) mm × 1e6×tpus/dt_ticks → diff × 5752428 / dt_us。 */
static constexpr int32_t kAS5600_SPEED_NUM = -5752428;

/* ==================== AS5600 编码器配置 ==================== */

/** @brief 全局 AS5600 软件 I2C 驱动实例，管理4通道编码器 */
AS5600_soft_IIC_many MC_AS5600;
/** @brief 各通道 AS5600 SCL 时钟线 GPIO 端口 */
static GPIO_TypeDef* const AS5600_SCL_PORT[4] = { GPIOB, GPIOB, GPIOB, GPIOB };
/** @brief 各通道 AS5600 SCL 时钟线 GPIO 引脚 */
static const uint16_t      AS5600_SCL_PIN [4] = { GPIO_Pin_15, GPIO_Pin_14, GPIO_Pin_13, GPIO_Pin_12 };
/** @brief 各通道 AS5600 SDA 数据线 GPIO 端口 */
static GPIO_TypeDef* const AS5600_SDA_PORT[4] = { GPIOD, GPIOC, GPIOC, GPIOC };
/** @brief 各通道 AS5600 SDA 数据线 GPIO 引脚 */
static const uint16_t      AS5600_SDA_PIN [4] = { GPIO_Pin_0, GPIO_Pin_15, GPIO_Pin_14, GPIO_Pin_13 };

static int64_t as5600_odometer_count[4] = {0,0,0,0};

/** @brief AS5600 每米对应的计数（= 1e6 µm ÷ 5.752428 µm/count = 173839.64 → 四舍五入 173840）
 *  @note 必须与 fx_units.h 的 FX_AS5600_CNT_PER_M 保持一致；
 *        取整引入相对误差 2.1e-6（12 m 处约 4 计数 / 25 µm），远低于滚轮直径本身的公差。 */
static constexpr uint32_t kAS5600_CNT_PER_M = 173840u;

/**
 * @brief 长度（微米）→ AS5600 计数（向上取整）
 * @param um 长度，单位微米，**要求 0 <= um <= 12000000（12 m）**
 * @note 【定点化】原实现为 ceil(meters / kAS5600_M_PER_CNT)。
 *       现改成整数版本：counts = ceil(um × 173840 / 1e6) = ceil(um × 173840 / 10^6)。
 *       为避免 um×173840 溢出 int32，把 um 拆成"米 / 毫米 / 微米余数"三段分别
 *       乘上 173840 后再累加（各自都远小于 2^31），最后统一向上取整。
 *       12 m 时最大中间量 = 12×173840×1000 ≈ 2.086e9 < 2^31 ✓
 */
static constexpr uint32_t distance_counts(int32_t um)
{
    return (uint32_t)(
        ((uint32_t)(um / kUmPerM) * kAS5600_CNT_PER_M * 1000u) +
        ((uint32_t)((um / kUsPerMs) % kUsPerMs) * kAS5600_CNT_PER_M) +
        (((uint32_t)(um % kUsPerMs) * kAS5600_CNT_PER_M) / 1000u) + 999u) / 1000u;
}

static inline uint32_t encoder_count(uint8_t ch)
{
    return (uint32_t)as5600_odometer_count[ch];
}

static inline uint32_t count_distance(uint32_t a, uint32_t b)
{
    const uint32_t d = a - b;
    return (d & 0x80000000u) ? 0u - d : d;
}

float Motion_control_filament_meters(uint8_t ch)
{
    if (ch >= kChCount) return 1.0f;

    /* 【定点化】原实现 1.0f - (float)count * kAS5600_M_PER_CNT 会拉进
     * __floatdisf(__floatundisf)/__mulsf3/__subsf3 共约 1.9 KB。
     * 这里改为纯整数换算 + IEEE-754 位运算装配（fx_meters_from_counts），
     * 一个软浮点助手都不引入。返回值仍是 float —— 因为该字段原样 memcpy
     * 进打印机总线报文，属协议硬约束。
     * 先把 int64 里程表限幅到 int32 可表示范围，避免转换溢出。 */
    int64_t c64 = as5600_odometer_count[ch];
    if (c64 >  2147483647LL) c64 =  2147483647LL;
    if (c64 < -2147483648LL) c64 = -2147483648LL;

    return fx_meters_from_counts((int32_t)c64);
}

/** @brief 各通道实时速度（µm/s），正值=送丝，负值=回退【定点化：原 mm/s 的 float】 */
int32_t speed_as5600[4] = {0, 0, 0, 0};
/* ===== AS5600 传感器健康监控（防飞车保护）===== */
/** @brief 传感器健康标志（1=正常, 0=故障），连续失败3次才标记故障 */
static uint8_t g_as5600_good[4]     = {0,0,0,0};
/** @brief 通信失败计数器 */
static uint8_t g_as5600_fail[4]     = {0,0,0,0};
/** @brief 通信成功连续计数器 */
static uint8_t g_as5600_okstreak[4] = {0,0,0,0};
/** @brief 故障触发阈值：连续失败3次 */
static constexpr uint8_t kAS5600_FAIL_TRIP   = 3;
/** @brief 恢复触发阈值：连续成功2次 */
static constexpr uint8_t kAS5600_OK_RECOVER  = 2;
/** @brief 检查指定通道 AS5600 是否健康 */
static inline bool AS5600_is_good(uint8_t ch) { return g_as5600_good[ch] != 0; }

/* ---- 回退运动参数 ---- */
/** @brief 回退快速速度（60 mm/s = 60000 µm/s） */
static constexpr int32_t PULL_V_FAST   = 60 * 1000;      // µm/s
/** @brief 回退末段慢速（12 mm/s = 12000 µm/s），防止冲击 */
static constexpr int32_t PULL_V_END    = 12 * 1000;      // µm/s
/** @brief 减速区长度（15 mm = 15000 µm），低于此距离开始减速 */
static constexpr int32_t PULL_RAMP_UM  = 15 * 1000;      // µm
/** @brief 回退最小 PWM（400 → 400000 毫 PWM），确保末端有足够"踢力" */
static constexpr int32_t PULL_PWM_MIN  = 400 * kPwmM;    // "kick"

/** @brief 各通道回退剩余距离（微米），用于减速区计算 */
static int32_t g_pull_remain_um[4]  = {0,0,0,0};
/** @brief 各通道回退目标速度（µm/s，负值=反向） */
static int32_t g_pull_speed_set[4] = {-PULL_V_FAST,-PULL_V_FAST,-PULL_V_FAST,-PULL_V_FAST};

/** @brief 各通道速度传感器电压偏移补偿量（微伏） */
int32_t MC_PULL_V_OFFSET[4]      = {0, 0, 0, 0};
/** @brief 各通道传感器最小电压（行程起点，微伏；默认 1.00 V） */
int32_t MC_PULL_V_MIN[4]         = {1000000, 1000000, 1000000, 1000000};
/** @brief 各通道传感器最大电压（行程终点，微伏；默认 2.00 V） */
int32_t MC_PULL_V_MAX[4]         = {2000000, 2000000, 2000000, 2000000};
/** @brief 各通道传感器极性（+1正常, -1反向） */
int8_t MC_PULL_POLARITY[4]     = {1, 1, 1, 1};
/** @brief 双微动开关"无料"电压阈值（微伏；默认 0.60 V） */
int32_t MC_DM_KEY_NONE_THRESH[4] = {600000, 600000, 600000, 600000};

/** @brief 各通道百分比读数（0~100 整数，四舍五入） */
uint8_t MC_PULL_pct[4]        = {50, 50, 50, 50};
/** @brief 各通道百分比读数（精确值，厘百分比 0~10000，用于PID） */
static int32_t MC_PULL_pct_cc[4] = {5000, 5000, 5000, 5000};

/** @brief 各通道传感器原始电压（已补偿偏移和极性，微伏） */
static int32_t  MC_PULL_stu_raw[4]        = {1650000, 1650000, 1650000, 1650000};
/** @brief 各通道传感器状态：-1=低压力, 0=中位, 1=高压力 */
static int8_t MC_PULL_stu[4]            = {0, 0, 0, 0};

/** @brief 各通道在线检测状态：0=无料, 1=双开关(到位), 2=仅外部开关(插入中) */
static uint8_t  MC_ONLINE_key_stu[4]    = {0, 0, 0, 0};
/** @brief 按键空采样发布计数（连续空采样达到阈值才判定无料，V11 防抖机制） */
static uint8_t g_key_empty[4] = {0,0,0,0};
static bool g_key_sample_fresh = false;
static constexpr uint8_t kKeyEmptyPublications = ADC_DMA_FILTER_BLOCKS + 1u;
/** @brief 低压力锁存标志（1=电机停止锁存），on_use下压力<40%时触发 */
static uint8_t  g_on_use_low_latch[4]   = {0, 0, 0, 0};
/** @brief 真实堵转锁存（1=上报0xF06F错误） */
static uint8_t  g_on_use_jam_latch[4]   = {0, 0, 0, 0};
/** @brief 高PWM推力累计时间（微秒），持续20秒触发非堵转停机 */
static uint32_t g_on_use_hi_pwm_us[4]   = {0u, 0u, 0u, 0u};

/**
 * @brief 设置状态指示灯RGB（带锁存保护）
 *        当g_on_use_low_latch有效时强制显示红色警告
 */
static inline __attribute__((always_inline)) void MC_STU_RGB_set_latch(uint8_t ch, uint8_t r, uint8_t g, uint8_t b, uint64_t now_ms, uint8_t blink)
{
    if (!g_on_use_low_latch[ch]) { MC_STU_RGB_set(ch, r, g, b); return; }

    if (!blink || (((now_ms / 1000ull) & 1ull) != 0ull))
        MC_STU_RGB_set(ch, 0xFFu, 0x00u, 0x00u);
    else
        MC_STU_RGB_set(ch, r, g, b);
}

/* ==================== 双微动开关检测逻辑 ==================== */

#if BMCU_DM_TWO_MICROSWITCH
/**
 * @brief 将双微动开关电压转换为数字状态
 *        0=无料, 1=双开关(到位), 2=仅外部开关(插入中), 3=其他
 */
static inline uint8_t dm_key_to_state(uint8_t ch, int32_t v_uv)
{
    const int32_t none_thr = MC_DM_KEY_NONE_THRESH[ch];

    if (v_uv < none_thr)  return 0u;   // none
    if (v_uv > 1700000)   return 1u;   // both（1.70 V）
    if (v_uv > 1400000)   return 2u;   // external only（1.40 V）
    return 3u;
}

/* ---- DM 自动加载状态机参数 ---- */
/** @brief Stage1 防抖时间（100ms） */
static constexpr uint64_t DM_AUTO_S1_DEBOUNCE_MS       = 100ull;   // 0.1s
/** @brief Stage1 超时时间（5s）：推料超时判定失败 */
static constexpr uint64_t DM_AUTO_S1_TIMEOUT_MS        = 5000ull;  // 5s
/** @brief Stage1 失败后回退时间（1.5s） */
static constexpr uint64_t DM_AUTO_S1_FAIL_RETRACT_MS   = 1500ull;  // 1.5s

/** @brief Stage2 目标推料距离（120mm = 120000 µm） */
static constexpr int32_t  DM_AUTO_S2_TARGET_UM         = 120 * 1000;   // µm
/** @brief Stage2 中止阈值（7500 厘% = 75%）：压力超过此值中止推料 */
static constexpr int32_t  DM_AUTO_BUF_ABORT_CC         = 7500;         // abort push
/** @brief Stage2 恢复阈值（5020 厘% = 50.2%）：回退到此值重新尝试 */
static constexpr int32_t  DM_AUTO_BUF_RECOVER_CC       = 5020;         // retract-to
/** @brief Stage2 失败后额外回退时间（1.5s） */
static constexpr uint64_t DM_AUTO_FAIL_EXTRA_MS        = 1500ull;  // extra retract
/** @brief 自动加载推料 PWM（900 → 900000 毫 PWM） */
static constexpr int32_t  DM_AUTO_PWM_PUSH             = 900 * kPwmM;  // push strength
/** @brief 自动加载回退 PWM（900 → 900000 毫 PWM） */
static constexpr int32_t  DM_AUTO_PWM_PULL             = 900 * kPwmM;  // retract strength
/** @brief 自动加载期间的PWM限制值（950 → 950000 毫 PWM） */
static constexpr int32_t  DM_AUTO_IDLE_LIM             = 950 * kPwmM;  // clamp only

/**
 * @brief DM 自动加载状态机状态枚举
 * IDLE→S1_DEBOUNCE→S1_PUSH→S2_PUSH→完成 | 任何失败→FAIL_RETRACT→IDLE
 */
enum : uint8_t
{
    DM_AUTO_IDLE = 0,              /**< 空闲，等待触发 */
    DM_AUTO_S1_DEBOUNCE,           /**< Stage1 防抖等待 */
    DM_AUTO_S1_PUSH,               /**< Stage1 推料至双开关位置 */
    DM_AUTO_S1_FAIL_RETRACT,       /**< Stage1 失败回退 */
    DM_AUTO_S2_PUSH,               /**< Stage2 推料至目标距离 */
    DM_AUTO_S2_RETRACT,            /**< Stage2 压力过大回退重试 */
    DM_AUTO_S2_FAIL_RETRACT,       /**< Stage2 失败回退（3次中止后） */
    DM_AUTO_S2_FAIL_EXTRA,         /**< Stage2 额外回退等待 */
};

/** @brief 各通道是否已加载（1=已加载，加载成功后置1） */
static uint8_t  dm_loaded[4]            = {1,1,1,1};
/** @brief 失败锁存（1=失败，需等微动归零后清除） */
static uint8_t  dm_fail_latch[4]        = {0,0,0,0};
/** @brief 自动加载状态机当前状态 */
static uint8_t  dm_auto_state[4]        = {0,0,0,0};
static uint8_t  dm_autoload_gate[4]     = {1,1,1,1}; // 0=allow autoload, 1=block until idle+confirmed ks==0
static uint8_t  dm_auto_try[4]          = {0,0,0,0};   // abort count (stage2)
static uint64_t dm_auto_t0_ms[4]        = {0ull,0ull,0ull,0ull};
static uint32_t dm_auto_remain_count[4] = {0,0,0,0};
static uint32_t dm_auto_last_count[4] = {0,0,0,0};
static uint8_t dm_idle_empty[4] = {0,0,0,0};
static constexpr uint32_t DM_AUTO_S2_TARGET_COUNT = distance_counts(DM_AUTO_S2_TARGET_UM);

static uint64_t dm_loaded_drop_t0_ms[4] = {0ull,0ull,0ull,0ull};

/* 复位双微动自动装载的状态机字段（三处重复逻辑共用） */
static void dm_auto_fields_reset(uint8_t ch)
{
    dm_auto_state[ch]    = DM_AUTO_IDLE;
    dm_auto_try[ch]      = 0u;
    dm_auto_t0_ms[ch]    = 0ull;
    dm_auto_remain_count[ch] = 0u;
    dm_auto_last_count[ch]   = 0u;
}

/* 复位状态机核心字段（保留 last_m 用于增量追踪） */
static void dm_auto_reset_state(uint8_t ch)
{
    dm_auto_state[ch]    = DM_AUTO_IDLE;
    dm_auto_try[ch]      = 0u;
    dm_auto_remain_count[ch] = 0u;
    dm_auto_t0_ms[ch]    = 0ull;
}

#endif

/* ==================== 自动卸料参数 ==================== */
/** @brief 自动卸料触发阈值（8000 厘% = 80%）：压力超过此值触发 */
static constexpr int32_t  AUTO_UNLOAD_START_CC      = 8000;
/** @brief 中性区下界（4500 厘% = 45%）：回到此值以下确认卸料成功 */
static constexpr int32_t  AUTO_UNLOAD_NEUTRAL_LO_CC = 4500;
/** @brief 中性区上界（5500 厘% = 55%）：在此区间内确认成功 */
static constexpr int32_t  AUTO_UNLOAD_NEUTRAL_HI_CC = 5500;
/** @brief 中止阈值（3500 厘% = 35%）：低于此值中止卸料 */
static constexpr int32_t  AUTO_UNLOAD_ABORT_CC      = 3500;
/** @brief 预备时间（1s）：触发后等待确认持续高压力 */
static constexpr uint64_t AUTO_UNLOAD_ARM_MS         = 1000ull;
/** @brief 最大执行时间（15s）：超时强制停止 */
static constexpr uint64_t AUTO_UNLOAD_MAX_MS         = 15000ull;
/** @brief 空载检测时间（1.5s）：压力低于中性区后持续此时间确认完成 */
static constexpr uint64_t AUTO_UNLOAD_EMPTY_MS       = 1500ull;
/** @brief 回退PWM（85%占空比 → 850000 毫 PWM） */
static constexpr int32_t  AUTO_UNLOAD_PWM_PULL       = 850 * kPwmM;

/** @brief 自动卸料预备状态标志（1=已触发等待确认） */
static uint8_t  auto_unload_arm[4]          = {0,0,0,0};
/** @brief 自动卸料激活状态标志（1=正在执行卸料） */
static uint8_t  auto_unload_active[4]       = {0,0,0,0};
/** @brief 自动卸料阻塞标志（1=已成功/失败，阻止再次触发） */
static uint8_t  auto_unload_blocked[4]      = {0,0,0,0};
/** @brief 预备状态开始时间戳（毫秒） */
static uint64_t auto_unload_arm_t0_ms[4]    = {0ull,0ull,0ull,0ull};
/** @brief 激活状态开始时间戳（毫秒） */
static uint64_t auto_unload_active_t0_ms[4] = {0ull,0ull,0ull,0ull};
/** @brief 空载检测开始时间戳（毫秒） */
static uint64_t auto_unload_empty_t0_ms[4]  = {0ull,0ull,0ull,0ull};

/** @brief 各通道是否物理插入（true=已插入，ADC 0.3V~3.0V） */
bool filament_channel_inserted[4]       = {false, false, false, false};

/** @brief PID 压力控制器 P 增益（原 25.0）。
 *  【定点化】误差单位为厘百分比，输出为毫 PWM：
 *        out_milli = 25 [PWM/%] × E[%] × 1000 = 25 × E_cc × 10 = 250 × E_cc
 *  故换算到定点域后 P 增益系数为 250。 */
static constexpr int32_t MC_PULL_PIDP_NUM = 250;

/** @brief 低压力死区下界（30%）：低于此值状态=-1 */
static constexpr int MC_PULL_DEADBAND_PCT_LOW  = 30;
/** @brief 高压力死区上界（70%）：高于此值状态=1 */
static constexpr int MC_PULL_DEADBAND_PCT_HIGH = 70;

/* ==================== 进料控制参数（按打印机型号区分） ====================== */
/**
 * 进料控制参数 —— 根据打印机型号选择不同参数组
 * BMCU_SOFT_LOAD=A1 Mini, BMCU_P1S=P1S, 默认=A1
 * Stage1:快速进料, Stage2:保持, ON_USE:打印中压力控制
 */
/* 【定点化】以下参数全部改为整数常量（编译期换算，不产生运行期浮点）：
 *   *_PCT      → 厘百分比 (0.01%)，例 90.0f → 9000
 *   *_PWM_*    → 毫 PWM (0.001)，例 480.0f → 480000
 *   用于换算的 (int32_t)(X * 100.0f) / (int32_t)(X * kPwmM) 都是 constexpr，
 *   由编译器在编译期完成，与旧 float 行为逐位一致。 */
#if BMCU_SOFT_LOAD
    /* ---- A1 Mini 参数 ---- */
    static constexpr int     MC_LOAD_S1_FAST_PCT       = 75;    /**< Stage1 快速进料百分比 */
    static constexpr int     MC_LOAD_S1_HARD_STOP_PCT  = 90;    /**< Stage1 硬限位（安全保护） */
    static constexpr int     MC_LOAD_S1_HARD_HYS       = 2;     /**< Stage1 硬限位滞回 */
    static constexpr int32_t MC_LOAD_S2_HOLD_TARGET_PCT    = 7500;   /**< Stage2 保持目标（厘%） */
    static constexpr int32_t MC_LOAD_S2_HOLD_BAND_LO_DELTA = 30;     /**< Stage2 区间下界偏移（厘%） */
    static constexpr int32_t MC_LOAD_S2_PUSH_START_PCT     = 5500;   /**< Stage2 开始推料百分比（厘%） */
    static constexpr int32_t MC_LOAD_S2_PWM_HI             = 480000; /**< Stage2 推料上限PWM（毫PWM） */
    static constexpr int32_t MC_LOAD_S2_PWM_LO             = 1000000;/**< Stage2 最大推力PWM（毫PWM） */
    static constexpr int32_t MC_ON_USE_TARGET_PCT    = 5200;   /**< ON_USE 压力目标（厘%） */
    static constexpr int32_t MC_ON_USE_BAND_LO_DELTA = 20;     /**< ON_USE 区间下界偏移（厘%） */
    static constexpr int32_t MC_ON_USE_BAND_HI_PCT   = 6000;   /**< ON_USE 区间上界（厘%） */
#elif BMCU_P1S  // P1S
    /* ---- P1S 参数 ---- */
    static constexpr int     MC_LOAD_S1_FAST_PCT       = 88;
    static constexpr int     MC_LOAD_S1_HARD_STOP_PCT  = 97;
    static constexpr int     MC_LOAD_S1_HARD_HYS       = 2;
    static constexpr int32_t MC_LOAD_S2_HOLD_TARGET_PCT    = 9500;
    static constexpr int32_t MC_LOAD_S2_HOLD_BAND_LO_DELTA = 100;
    static constexpr int32_t MC_LOAD_S2_PUSH_START_PCT     = 8800;
    static constexpr int32_t MC_LOAD_S2_PWM_HI             = 550000;
    static constexpr int32_t MC_LOAD_S2_PWM_LO             = 1000000;
    static constexpr int32_t MC_ON_USE_TARGET_PCT    = 5400;
    static constexpr int32_t MC_ON_USE_BAND_LO_DELTA = 20;
    static constexpr int32_t MC_ON_USE_BAND_HI_PCT   = 6500;
#else        // A1
    /* ---- A1 默认参数 ---- */
    static constexpr int     MC_LOAD_S1_FAST_PCT       = 85;
    static constexpr int     MC_LOAD_S1_HARD_STOP_PCT  = 95;
    static constexpr int     MC_LOAD_S1_HARD_HYS       = 2;
    static constexpr int32_t MC_LOAD_S2_HOLD_TARGET_PCT    = 9000;
    static constexpr int32_t MC_LOAD_S2_HOLD_BAND_LO_DELTA = 30;
    static constexpr int32_t MC_LOAD_S2_PUSH_START_PCT     = 8000;
    static constexpr int32_t MC_LOAD_S2_PWM_HI             = 480000;
    static constexpr int32_t MC_LOAD_S2_PWM_LO             = 1000000;
    static constexpr int32_t MC_ON_USE_TARGET_PCT    = 5200;
    static constexpr int32_t MC_ON_USE_BAND_LO_DELTA = 20;
    static constexpr int32_t MC_ON_USE_BAND_HI_PCT   = 6000;
#endif
// ====================================================

static constexpr uint64_t ON_USE_HANDOFF_MS = 10000ull;
/** @brief 交接上限滞回（25 厘% = 0.25%） */
static constexpr int32_t ON_USE_HANDOFF_CEILING_HYS_CC = 25;

/* ==================== 校准重置参数 ==================== */
/** @brief 校准重置长按时间（5秒） */
static constexpr uint32_t CAL_RESET_HOLD_MS     = 5000;
/** @brief 触发校准重置的百分比阈值 */
static constexpr int      CAL_RESET_PCT_THRESH  = 15;
/** @brief 校准重置电压偏移容差（0.10 V = 100000 µV） */
static constexpr int32_t  CAL_RESET_V_DELTA_UV  = 100000;
/** @brief 校准重置最小电压容差（0.03 V = 30000 µV） */
static constexpr int32_t  CAL_RESET_NEAR_MIN_UV = 30000;

/** @brief 当前被按住的通道号（-1=无） */
static int      g_hold_ch = -1;
/** @brief 按住开始时的tick时间戳 */
static uint32_t g_hold_t0_ticks = 0;

/**
 * @brief 各通道最后退出on_use的时间戳（毫秒）
 *        0=从未进入, 1=曾经进入, 其他=实际退出时间
 *        用于自动换料后防止挤出机继续夹持
 */
static uint64_t g_last_on_use_exit_ms[4] = {0,0,0,0};

/** @brief 外部声明：RGB LED更新函数 */
extern void RGB_update();

/** @brief 检查所有4通道是否都无耗材 */
static inline bool all_no_filament()
{
    return ((MC_ONLINE_key_stu[0] | MC_ONLINE_key_stu[1] | MC_ONLINE_key_stu[2] | MC_ONLINE_key_stu[3]) == 0);
}

/** @brief 所有通道蓝色LED闪烁3秒（校准重置前视觉提示） */
static void blink_all_blue_3s()
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = 3000u * tpm;

    while ((uint32_t)(time_ticks32() - t0) < dt)
    {
        const uint32_t now_t = time_ticks32();
        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);

        const bool on = (((elapsed_ms / 150u) & 1u) == 0u);
        for (uint8_t ch = 0; ch < kChCount; ch++)
            MC_PULL_ONLINE_RGB_set(ch, 0, 0, on ? 0x10 : 0);

        RGB_update();
        delay(20);
    }

    for (uint8_t ch = 0; ch < kChCount; ch++)
        MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
}

/**
 * @brief 执行校准重置并重启MCU
 *        停止电机→蓝色闪烁3秒→清除Flash→NVIC系统复位
 */
static void calibration_reset_and_reboot()
{
    if (!Flash_background_idle()) return;
    for (uint8_t i = 0; i < kChCount; i++) Motion_control_set_PWM(i, 0);

    bus_shutdown();
    const bool cal_cleared = MC_PULL_calibration_clear();
    const bool motion_cleared = cal_cleared && Flash_Motion_clear();
    if (!motion_cleared) {
        g_hold_ch = -1;
        g_hold_t0_ticks = 0u;
        for (uint8_t i = 0; i < kChCount; i++) MC_PULL_ONLINE_RGB_set(i, 0x18, 0, 0);
        RGB_update();
        bus_init();
        return;
    }
    blink_all_blue_3s();
    NVIC_SystemReset();
}

/**
 * @brief 将传感器电压转换为压力百分比（厘百分比 0~10000）
 *        分段线性映射，以 1.65 V 为中位点：
 *        <=1.65V: 从 vmin 线性映射到 0%~50%, >1.65V: 从 1.65V 映射到 50%~100%
 * @param ch   通道号
 * @param v_uv 已补偿偏移与极性的电压（微伏）
 * @return 压力百分比，单位厘百分比（0.01%），范围 0~10000
 * @note 【定点化】分母 den 已保证 >= 50000（0.05 V）。为避免
 *       (v - vmin) × 5000 溢出 int32，先把分子分母同除 16 等比缩小，
 *       由此引入的绝对误差 < 5000×16/den ≤ 1.6 厘%（即 < 0.016%）。
 */
static int32_t pull_cc_from_uv(uint8_t ch, int32_t v_uv)
{
    constexpr int32_t c = 1650000;          /* 1.65 V 中位点（µV） */

    int32_t vmin = MC_PULL_V_MIN[ch];
    int32_t vmax = MC_PULL_V_MAX[ch];

    if (vmin > 1600000) vmin = 1600000;     /* 1.60 V */
    if (vmax < 1700000) vmax = 1700000;     /* 1.70 V */
    if (vmax <= (vmin + 100000)) { vmin = 1550000; vmax = 1750000; }  /* 0.10 V 容差 */

    int32_t pos_cc;
    if (v_uv <= c)
    {
        int32_t den = c - vmin;             /* 单位 µV */
        if (den < 50000) den = 50000;       /* 0.05 V 下限 */
        pos_cc = (int32_t)(((int32_t)((v_uv - vmin) >> 4) * 5000) / (den >> 4));
    }
    else
    {
        int32_t den = vmax - c;
        if (den < 50000) den = 50000;
        pos_cc = 5000 + (int32_t)(((int32_t)((v_uv - c) >> 4) * 5000) / (den >> 4));
    }

    if (pos_cc < 0) pos_cc = 0;
    if (pos_cc > 10000) pos_cc = 10000;
    return pos_cc;
}

/** @brief 根据极性反转电压：当 POLARITY<0 时返回 3.3V - v（微伏）
 *  【定点化】原 float 版为 3.30f - v，现全部按微伏整数运算 */
static inline int32_t pull_v_apply_polarity(uint8_t ch, int32_t v_uv)
{
    if (MC_PULL_POLARITY[ch] < 0) return 3300000 - v_uv;
    return v_uv;
}

/**
 * @brief 检测4个通道是否物理插入
 *        通过ADC读取电压，判断是否在0.3V~3.0V有效范围内
 *        多次采样取平均提高可靠性
 */
void MC_PULL_detect_channels_inserted()
{
    if (!ADC_DMA_is_inited())
    {
        for (uint8_t ch = 0; ch < kChCount; ch++) filament_channel_inserted[ch] = false;
        return;
    }

    ADC_DMA_gpio_analog();
    ADC_DMA_filter_reset();
    (void)ADC_DMA_wait_full();

    constexpr uint8_t idx[kChCount] = {6,4,2,0};
    constexpr int N = 16;
    /* 【定点化】累加微伏：16 × 3,250,000 = 5.2e7，int32 足够 */
    int32_t s[kChCount] = {0,0,0,0};

    for (int i = 0; i < N; i++)
    {
        const int32_t *v = ADC_DMA_get_value();
        for (uint8_t ch = 0; ch < kChCount; ch++) s[ch] += v[idx[ch]];
        delay(2);
    }

    constexpr int32_t VMIN = 300000;    /* 0.30 V */
    constexpr int32_t VMAX = 3000000;   /* 3.00 V */

    for (uint8_t ch = 0; ch < kChCount; ch++)
    {
        const int32_t a = s[ch] / N;    /* N=16 为常量除法 */
        filament_channel_inserted[ch] = (a > VMIN) && (a < VMAX);
    }
}

/** @brief 初始化ADC传感器读取（插入检测+电压读取） */
static inline void MC_PULL_ONLINE_init()
{
    MC_PULL_detect_channels_inserted();
}

/**
 * @brief 读取ADC传感器数据，更新各通道电压/百分比/状态
 *        从DMA缓冲区读取8通道数据（4速度传感器+4微动开关）
 *        双微动开关模式下额外检测Buffer Gesture Load手势
 */
static inline void MC_PULL_ONLINE_read(uint32_t now_ticks)
{
    const int32_t *data = ADC_DMA_get_value();
    static uint32_t last_generation = 0u;
    const uint32_t generation = ADC_DMA_generation();
    g_key_sample_fresh = generation != last_generation && ADC_DMA_ready();
    last_generation = generation;

    /* 映射 ADC 通道 -> 耗材通道（交错排列：偶数=速度传感器，奇数=微动开关）
     * 【定点化】ADC 输出即 int32 微伏，全程整数运算，不再经过 float */
    MC_PULL_stu_raw[3] = pull_v_apply_polarity(3u, data[0] + MC_PULL_V_OFFSET[3]);
    const int32_t key3 = data[1];

    MC_PULL_stu_raw[2] = pull_v_apply_polarity(2u, data[2] + MC_PULL_V_OFFSET[2]);
    const int32_t key2 = data[3];

    MC_PULL_stu_raw[1] = pull_v_apply_polarity(1u, data[4] + MC_PULL_V_OFFSET[1]);
    const int32_t key1 = data[5];

    MC_PULL_stu_raw[0] = pull_v_apply_polarity(0u, data[6] + MC_PULL_V_OFFSET[0]);
    const int32_t key0 = data[7];

#if BMCU_DM_TWO_MICROSWITCH
    const int32_t keyv[4] = { key0, key1, key2, key3 };

    /* --- Buffer Gesture Load（缓冲手势加载检测）---
     * 检测耗材插入的手势序列：空闲→低电压→恢复→中位确认
     * 防止误触发自动加载
     */
    static uint32_t gst_t0_ticks[4]     = {0,0,0,0};
    static uint8_t  gst_step[4]         = {0,0,0,0};      // 0=idle, 1=wait_low, 2=wait_return
    static bool     gst_active[4]       = {false,false,false,false};
    static uint32_t gst_act_t0_ticks[4] = {0,0,0,0};

    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    const uint32_t T100  = 100u  * tpm;
    const uint32_t T2000 = 2000u * tpm;
    const uint32_t T5500 = 5500u * tpm;

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (!filament_channel_inserted[i])
        {
            gst_step[i] = 0;
            gst_active[i] = false;
            gst_t0_ticks[i] = 0;
            gst_act_t0_ticks[i] = 0;
            MC_ONLINE_key_stu[i] = 0u;
            continue;
        }

        if (dm_fail_latch[i])
        {
            gst_step[i] = 0;
            gst_active[i] = false;
        }

        if (!gst_active[i])
        {
            const int32_t pct_cc = pull_cc_from_uv(i, MC_PULL_stu_raw[i]);

            if (gst_step[i] == 0)
            {
                if (pct_cc < 1000) { gst_step[i] = 1; gst_t0_ticks[i] = now_ticks; }
            }
            else if (gst_step[i] == 1)
            {
                if (pct_cc > 1500) { gst_step[i] = 0; }
                else if ((uint32_t)(now_ticks - gst_t0_ticks[i]) >= T100)
                {
                    gst_step[i] = 2;
                }
            }
            else
            {
                if ((uint32_t)(now_ticks - gst_t0_ticks[i]) > T2000)
                {
                    gst_step[i] = 0;
                }
                else if (pct_cc >= 4500 && pct_cc <= 5500)
                {
                    gst_active[i] = true;
                    gst_act_t0_ticks[i] = now_ticks;
                    gst_step[i] = 0;
                }
            }
        }

        if (gst_active[i])
        {
            if (keyv[i] > 1700000) gst_active[i] = false;
            else if ((uint32_t)(now_ticks - gst_act_t0_ticks[i]) > T5500) gst_active[i] = false;
        }

        const uint8_t phys = dm_key_to_state(i, keyv[i]);
        uint8_t state = phys;

        if (gst_active[i] && (phys == 0u)) state = 2u;

        MC_ONLINE_key_stu[i] = state;
    }
    /* --- End Buffer Gesture Load --- */
#else
    /* 简单模式：仅根据微动开关电压判断在线（>1.7V=在线） */
    /* 【定点化】1.7 V = 1700000 µV */
    MC_ONLINE_key_stu[3] = (filament_channel_inserted[3] && (key3 > 1700000)) ? 1u : 0u;
    MC_ONLINE_key_stu[2] = (filament_channel_inserted[2] && (key2 > 1700000)) ? 1u : 0u;
    MC_ONLINE_key_stu[1] = (filament_channel_inserted[1] && (key1 > 1700000)) ? 1u : 0u;
    MC_ONLINE_key_stu[0] = (filament_channel_inserted[0] && (key0 > 1700000)) ? 1u : 0u;
#endif


    /* 更新各通道百分比和状态 */
    for (uint8_t i = 0; i < kChCount; i++)
    {
        const bool ins = filament_channel_inserted[i];

        /* 未插入通道 → 重置为中位状态 */
        if (!ins)
        {
            MC_ONLINE_key_stu[i] = 0;
            MC_PULL_pct_cc[i] = 5000;
            MC_PULL_pct[i]   = 50;
            MC_PULL_stu[i]   = 0;
            continue;
        }

        const int32_t pct_cc = pull_cc_from_uv(i, MC_PULL_stu_raw[i]);
        MC_PULL_pct_cc[i] = pct_cc;

        /* 厘百分比 → 整数百分比，四舍五入（+50 再整除 100，与原 +0.5f 行为等价） */
        int pct = (int)((pct_cc + 50) / 100);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        MC_PULL_pct[i] = (uint8_t)pct;

    /* 基于死区阈值判断压力状态 */
    if      (pct > MC_PULL_DEADBAND_PCT_HIGH) MC_PULL_stu[i] = 1;
    else if (pct < MC_PULL_DEADBAND_PCT_LOW)  MC_PULL_stu[i] = -1;
    else                                      MC_PULL_stu[i] = 0;
    }

    /* 将当前活跃通道的压力值上报给打印机 */
    auto &A = ams[motion_control_ams_num];
    const uint8_t num = A.now_filament_num;

    if ((num != 0xFF) && (num < kChCount) && filament_channel_inserted[num])
    {
        const uint8_t pct = MC_PULL_pct[num];
            const uint32_t hi = (pct > 50u) ? (uint32_t)(pct - 50u) : 0u;
            A.pressure = (int)((hi * 65535u) / 50u);
    }
    else
    {
        A.pressure = 0xFFFF;
    }
}

/* ==================== Flash 保存/读取 ==================== */

/**
 * @brief 运动控制校准数据保存结构体
 *        存储电机方向和双微动开关阈值到Flash
 */
struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];       /**< 各通道电机方向（+1或-1） */
    uint32_t check;                  /**< 校验和（0x40614061=有效） */
    uint8_t dm_key_none_cv[4];       /**< "无料"阈值（centi-volt格式） */
} Motion_control_data_save;

/** @brief 设置保存数据的默认值（方向=0未校准, 阈值=60 centi-volt） */
static inline void Motion_control_defaults()
{
    for (uint8_t i = 0; i < kChCount; i++)
    {
        Motion_control_data_save.Motion_control_dir[i] = 0;
        Motion_control_data_save.dm_key_none_cv[i] = 60u;
    }

    Motion_control_data_save.check = 0x40614061u;
}

/** @brief 将保存的校准数据应用到运行时变量 */
static inline void Motion_control_apply_saved()
{
    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (Motion_control_data_save.dm_key_none_cv[i] < 60u)
            Motion_control_data_save.dm_key_none_cv[i] = 60u;

        if (!MC_PULL_calibration_is_valid(i)) MC_DM_KEY_NONE_THRESH[i] = 600000;  /* 0.60 V */
    }
}

/** @brief 从Flash读取校准数据，失败或校验不匹配时用默认值 */
static inline bool Motion_control_read()
{
    Motion_control_defaults();

    if (!Flash_Motion_read(&Motion_control_data_save, (uint16_t)sizeof(Motion_control_save_struct)))
    {
        Motion_control_apply_saved();
        return false;
    }

    if (Motion_control_data_save.check != 0x40614061u)
    {
        Motion_control_defaults();
        Motion_control_apply_saved();
        return false;
    }

    Motion_control_apply_saved();
    return true;
}

/** @brief 将校准数据保存到Flash */
static inline bool Motion_control_save()
{
    Motion_control_data_save.check = 0x40614061u;

    for (uint8_t i = 0; i < kChCount; i++)
    {
        uint8_t cv = dm_key_v_to_centi_ceil(MC_DM_KEY_NONE_THRESH[i]);
        if (cv < 60u) cv = 60u;
        Motion_control_data_save.dm_key_none_cv[i] = cv;
    }

    return Flash_Motion_write(&Motion_control_data_save, (uint16_t)sizeof(Motion_control_save_struct));
}

/** @brief 保存双微动开关"无料"阈值到Flash（合并现有数据） */
bool Motion_control_save_dm_key_none_thresholds(void)
{
    int32_t thr[4];
    for (uint8_t i = 0; i < kChCount; i++)
        thr[i] = MC_DM_KEY_NONE_THRESH[i];

    (void)Motion_control_read();

    for (uint8_t i = 0; i < kChCount; i++)
        MC_DM_KEY_NONE_THRESH[i] = thr[i];

    return Motion_control_save();
}

/* ==================== PID 控制器 ==================== */

/**
 * @class MOTOR_PID
 * @brief 增量式PID控制器，用于电机速度/压力闭环控制
 *        output = P*error + ∫(I*error*dt) + D*(error-last_error)/dt
 *        带积分抗饱和和输出限幅
 *
 * 【定点化】
 *   全部状态与增益改为 int32，误差单位由调用方按"已放大的自然单位"给出：
 *     · 速度环：误差单位 µm/s（原 mm/s × 1000），输出单位 毫 PWM
 *     · 压力环：误差单位 厘%（原 % × 100），输出单位 毫 PWM
 *   原 float 参数 (P, I, D) 的语义被拆成三个整数：
 *     P_num     —— 输出(毫PWM) = P_num × E
 *     I_num/I_den —— 积分增量 ΔI = I_num × E × dt(100µs 为单位) / I_den
 *   D（微分）在本工程中两处实例化都取 0（MOTOR_PID(2,20,0) 与
 *   MOTOR_PID(25,0,0)），因此直接去掉微分项，避免 (E-E_last)/dt 的除法。
 *
 *   换算依据（见 _MOTOR_CONTROL 中 PID_speed / PID_pressure 的注释）：
 *     PID_speed    原 P=2, I=20 → P_num=2,    I_num=1, I_den=500
 *     PID_pressure 原 P=25, I=0 → P_num=250,  I_num=0（无积分）
 */
class MOTOR_PID
{
    int32_t P_num = 0;      /**< 比例增益：输出(毫PWM) = P_num × 误差 */
    int32_t I_num = 0;      /**< 积分增益分子（0 = 无积分） */
    int32_t I_den = 1;      /**< 积分增益分母 */
    int32_t I_save = 0;     /**< 积分累积值（毫 PWM，带抗饱和） */

    int32_t pid_MAX = PWM_lim * kPwmM;                        /**< 输出上限 */
    int32_t pid_MIN = -PWM_lim * kPwmM;                       /**< 输出下限 */
    int32_t pid_range = PWM_lim * kPwmM;                      /**< 积分抗饱和范围 */

    /** @brief 积分输入误差的钳位上限（µV 级误差也不可能到这个量级，纯防溢出） */
    static constexpr int32_t E_INT_CLAMP = 400000;

public:
    MOTOR_PID() = default;

    /** @brief 构造函数，使用指定整数增益初始化 */
    MOTOR_PID(int32_t P_set, int32_t I_set_num, int32_t I_set_den)
    {
        init_PID(P_set, I_set_num, I_set_den);
    }

    /** @brief 初始化PID整数增益并清零状态 */
    void init_PID(int32_t P_set, int32_t I_set_num, int32_t I_set_den)
    {
        P_num = P_set;
        I_num = I_set_num;
        I_den = (I_set_den > 0) ? I_set_den : 1;
        I_save = 0;
    }

    /**
     * @brief 计算PID输出（带积分抗饱和和输出限幅）
     * @param E        误差（速度环：µm/s；压力环：厘%）
     * @param dt_us100 时间步长，单位 100 µs（= time_us / 100）
     * @return 输出，单位毫 PWM，已限幅到 ±PWM_lim×1000
     */
    int32_t caculate(int32_t E, uint32_t dt_us100)
    {
        if (I_num != 0)
        {
            /* 误差钳位后参与积分：|E_clamped| × dt(≤2000) × I_num(≤1) < 2^31 */
            int32_t Ei = E;
            if (Ei >  E_INT_CLAMP) Ei =  E_INT_CLAMP;
            if (Ei < -E_INT_CLAMP) Ei = -E_INT_CLAMP;

            I_save += (int32_t)((I_num * Ei * (int32_t)dt_us100) / I_den);
            if (I_save >  pid_range) I_save =  pid_range;
            if (I_save < -pid_range) I_save = -pid_range;
        }

        int32_t out = P_num * E + I_save;

        if (out > pid_MAX) out = pid_MAX;
        if (out < pid_MIN) out = pid_MIN;

        return out;
    }

    /** @brief 清零积分历史（状态切换时调用） */
    void clear()
    {
        I_save = 0;
    }
};

/* ==================== 运动状态枚举 ==================== */

/**
 * @brief 电机运动模式枚举（9种工作模式）
 */
enum class filament_motion_enum
{
    filament_motion_send,                   /**< 送丝：BMCU→打印头 */
    filament_motion_redetect,               /**< 重新检测：耗材脱离后重新推入 */
    filament_motion_pull,                   /**< 回退：打印头→BMCU */
    filament_motion_stop,                   /**< 停止 */
    filament_motion_before_on_use,          /**< 打印前准备：维持适当压力 */
    filament_motion_stop_on_use,            /**< 打印中停止 */
    filament_motion_pressure_ctrl_on_use,   /**< 打印中压力控制（PID持续调节） */
    filament_motion_pressure_ctrl_idle,     /**< 空闲时压力控制 */
    filament_motion_before_pull_back,       /**< 回退前准备：先释放压力 */
};


/* ==================== 电机控制类 ==================== */

/**
 * @class _MOTOR_CONTROL
 * @brief 单通道电机控制器，封装完整运动控制逻辑
 *        包含：运动状态机、速度/压力PID、PWM输出、堵转检测、安全联锁
 */
class _MOTOR_CONTROL
{
public:
    filament_motion_enum motion = filament_motion_enum::filament_motion_stop;
    int CHx = 0;

    uint8_t pwm_zeroed = 1;

    uint64_t motor_stop_time = 0;

    /* 【定点化】哨兵值由 -1.0f 改为 -100（= -1.0 %，单位厘百分比）
     * 两个阈值量都只在 >= 0 时被视为有效，-100 与 -1.0f 的判定效果一致 */
    int32_t  post_sendout_retract_thresh_pct = -100;
    uint8_t  retract_hys_active = 0;
    int32_t  on_use_handoff_ceiling_pct = -100;
    uint64_t on_use_handoff_t0_ms = 0ull;

    uint64_t send_start_ms = 0;
    uint32_t send_start_count = 0u;
    uint8_t  send_len_abort = 0;

    uint64_t pull_start_ms = 0;

    bool send_stop_latch = false;

    /* 【定点化】整数增益换算（详见 MOTOR_PID 类注释）：
     *   PID_speed    原 (P=2, I=20, D=0)
     *       · P：误差=µm/s，输出毫PWM = 2 × E_um
     *       · I：ΔI = E_um × dt_100µs / 500，等价于原 20 × E_mm/s × dt_s
     *   PID_pressure 原 (P=25, I=0, D=0)
     *       · P：误差=厘%，输出毫PWM = 250 × E_cc（即 25 PWM 每 %） */
    MOTOR_PID PID_speed    = MOTOR_PID(2, 1, 500);
    MOTOR_PID PID_pressure = MOTOR_PID(MC_PULL_PIDP_NUM, 0, 1);

    int32_t pwm_zero = 500 * kPwmM;    /**< 空闲基础 PWM（毫 PWM，原 500） */
    /** @brief 电机方向：-1 / 0 / +1（原为 float，取值实际只有这三个）
     *  【定点化】改整数后 `dir != 0`、`x * dir < 0` 等判定全部直接做整数比较 */
    int32_t dir = 0;

    static int32_t x_prev[4];

    bool  send_hard = false;

    /** @brief 构造函数 */
    _MOTOR_CONTROL(int _CHx) : CHx(_CHx) {}

    /** @brief 设置空闲基础PWM值（毫 PWM） */
    void set_pwm_zero(int32_t _pwm_zero) { pwm_zero = _pwm_zero; }

    /** @brief 设置运动模式（使用当前时间） */
    void set_motion(filament_motion_enum _motion, uint64_t over_time)
    {
        set_motion(_motion, over_time, time_ms_fast());
    }

    /**
     * @brief 设置运动模式（指定时间戳）
     *        执行状态切换初始化：清零PID、设置停止时间、管理on_use时间戳
     */
    void set_motion(filament_motion_enum _motion, uint64_t over_time, uint64_t time_now)
    {
        motor_stop_time = (_motion == filament_motion_enum::filament_motion_stop) ? 0 : (time_now + over_time);

        if (motion == _motion) return;

        const filament_motion_enum prev = motion;
        motion = _motion;

        if ((_motion != filament_motion_enum::filament_motion_pressure_ctrl_on_use) &&
            g_on_use_low_latch[CHx] && !g_on_use_jam_latch[CHx])
        {
            g_on_use_low_latch[CHx] = 0u;
            g_on_use_hi_pwm_us[CHx] = 0u;
        }

        pwm_zeroed = 0;

        if (_motion == filament_motion_enum::filament_motion_send) {
            send_start_ms = time_now;
            send_stop_latch = false;
            send_len_abort = 0;
            send_start_count = encoder_count(CHx);
        }

        if (_motion == filament_motion_enum::filament_motion_pull) {
            pull_start_ms = time_now;
        }

        if (prev == filament_motion_enum::filament_motion_send &&
            _motion != filament_motion_enum::filament_motion_send)
        {
            send_start_ms = 0;
            send_stop_latch = false;
            send_len_abort = 0;
            send_start_count = 0u;
        }

        if (prev == filament_motion_enum::filament_motion_pull &&
            _motion != filament_motion_enum::filament_motion_pull)
        {
            pull_start_ms = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            if (g_last_on_use_exit_ms[CHx] == 0) g_last_on_use_exit_ms[CHx] = 1;
        }

        if (prev == filament_motion_enum::filament_motion_pressure_ctrl_on_use &&
            _motion != filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            g_last_on_use_exit_ms[CHx] = time_now;
        }

        if (_motion == filament_motion_enum::filament_motion_send ||
            _motion == filament_motion_enum::filament_motion_pull)
        {
            g_last_on_use_exit_ms[CHx] = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_send)
        {
            send_hard = false;
        }

        if (prev == filament_motion_enum::filament_motion_send &&
            _motion != filament_motion_enum::filament_motion_send)
        {
            send_hard = false;
        }

        PID_speed.clear();
        PID_pressure.clear();

        const bool keep_pwm =
            (prev == filament_motion_enum::filament_motion_send) &&
            (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use);

        if (_motion == filament_motion_enum::filament_motion_send)
        {
            post_sendout_retract_thresh_pct = -100;
            retract_hys_active = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_before_on_use || _motion == filament_motion_enum::filament_motion_stop_on_use)
        {
            int32_t p = MC_PULL_pct_cc[CHx];
            if (p < 0) p = 0;
            if (p > 10000) p = 10000;
            post_sendout_retract_thresh_pct = p;
            retract_hys_active = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            retract_hys_active = 0;
            int32_t p = MC_PULL_pct_cc[CHx];
            if (p < 0) p = 0;
            if (p > 10000) p = 10000;

            post_sendout_retract_thresh_pct = p;

            if (prev == filament_motion_enum::filament_motion_before_on_use ||
                prev == filament_motion_enum::filament_motion_stop_on_use ||
                prev == filament_motion_enum::filament_motion_send)
            {
                const int32_t band_hi = MC_ON_USE_BAND_HI_PCT;
                if (p > band_hi)
                {
                    on_use_handoff_ceiling_pct = p;
                    on_use_handoff_t0_ms = time_now;
                }
                else
                {
                    on_use_handoff_ceiling_pct = -100;
                    on_use_handoff_t0_ms = 0ull;
                }
            }
            else
            {
                on_use_handoff_ceiling_pct = -100;
                on_use_handoff_t0_ms = 0ull;
            }
        }
        else if (prev == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            post_sendout_retract_thresh_pct = -100;
            retract_hys_active   = 0;
            on_use_handoff_ceiling_pct   = -100;
            on_use_handoff_t0_ms = 0ull;
        }

        if (_motion == filament_motion_enum::filament_motion_pull)
        {
            post_sendout_retract_thresh_pct = -100;
            retract_hys_active = 0;
        }

        if (!keep_pwm)
        {
            x_prev[CHx] = 0;
        }
        else
        {
            if (x_prev[CHx] >  600 * kPwmM) x_prev[CHx] =  600 * kPwmM;
            if (x_prev[CHx] < -850 * kPwmM) x_prev[CHx] = -850 * kPwmM;
        }
    }

    /** @brief 获取当前运动模式 */
    filament_motion_enum get_motion() { return motion; }

    /**
     * @brief 保持进料压力控制（Stage2/hold_load通用）
     *        压力>阈值：滞回回退, 压力在推料区：线性推料, 正常：停止
     */
    static inline void hold_load(
        int32_t pct_cc,
        int32_t dir,
        MOTOR_PID &PID_pressure,
        int32_t &post_sendout_retract_thresh_pct,
        uint8_t &retract_hys_active,
        int32_t &x,
        bool  &on_use_need_move,
        int32_t &on_use_abs_err,
        bool  &on_use_linear
    )
    {
        /* 【定点化】pct/thresh/x 的单位分别是 厘%、厘%、毫 PWM；
         * dir 为 -1/0/+1 的整数方向；on_use_abs_err 单位 厘%。 */
        constexpr int32_t hold_target = MC_LOAD_S2_HOLD_TARGET_PCT;   /* 厘% */

        int32_t thresh = post_sendout_retract_thresh_pct;
        if (thresh < hold_target) thresh = hold_target;

        if (pct_cc > thresh)
        {
            const int32_t target = thresh;

            const int32_t start_retract = target + 25;   /* +0.25 % = 25 厘% */
            const int32_t stop_retract  = target;

            retract_hys_active = hyst_u8(retract_hys_active, pct_cc, start_retract, stop_retract);

            if (!retract_hys_active)
            {
                x = 0;
                PID_pressure.clear();
                on_use_need_move = false;
                on_use_abs_err   = 0;
                on_use_linear    = false;
            }
            else
            {
                const int32_t err = pct_cc - target;
                on_use_need_move = true;
                on_use_abs_err   = err;
                on_use_linear    = false;

                const int32_t mag = retract_mag_from_err(err, 850 * kPwmM);

                x = dir * mag;
                if (x * dir < 0) x = 0;
            }
        }
        else
        {
            retract_hys_active = 0;

            constexpr int32_t push_hi_pct    = hold_target - MC_LOAD_S2_HOLD_BAND_LO_DELTA;
            constexpr int32_t push_start_pct = MC_LOAD_S2_PUSH_START_PCT;
            constexpr int32_t pwm_hi         = MC_LOAD_S2_PWM_HI;
            constexpr int32_t pwm_lo         = MC_LOAD_S2_PWM_LO;
            /* slope 单位：毫 PWM / 厘%（与 pwm_hi/pwm_lo、push_*_pct 同量纲，直接相除） */
            constexpr int32_t slope          = (pwm_lo - pwm_hi) / (push_hi_pct - push_start_pct);

            if (pct_cc >= push_hi_pct)
            {
                x = 0;
                PID_pressure.clear();
                on_use_need_move = false;
                on_use_abs_err   = 0;
                on_use_linear    = false;
            }
            else
            {
                int32_t pwm;
                if (pct_cc <= push_start_pct) pwm = pwm_lo;
                else                          pwm = pwm_hi + (push_hi_pct - pct_cc) * slope;

                x = -dir * pwm;
                PID_pressure.clear();

                on_use_need_move = true;
                on_use_abs_err   = hold_target - pct_cc;
                on_use_linear    = true;
            }
        }
    }

    /**
     * @brief 电机控制主运行函数（每通道每循环调用一次）
     *        根据运动模式执行：stop/send/pull/on_use/idle等控制逻辑
     *        包含：软启动、硬限位保护、堵转检测、PWM斜率限制
     * @param time_us 时间步长（微秒）
     * @param now_ms 当前时间（毫秒）
     * @note 【定点化】时间步长由"秒(float)"改为"微秒(uint32)"：
     *       内部需要的是 dt，用整数微秒即可，既避免 float，
     *       也避免 int64→float 转换（__floatdisf/__floatundisf）。
     */
    void run(uint32_t time_us, uint64_t now_ms)
    {
        if (!MC_PULL_calibration_is_valid((uint8_t)CHx)) {
            PID_speed.clear();
            PID_pressure.clear();
            pwm_zeroed = 1;
            x_prev[CHx] = 0;
            Motion_control_set_PWM(CHx, 0);
            return;
        }

        if (motion == filament_motion_enum::filament_motion_stop &&
            motor_stop_time == 0 &&
            pwm_zeroed)
            return;

        if (motion != filament_motion_enum::filament_motion_stop &&
            motor_stop_time != 0 &&
            now_ms > motor_stop_time)
        {
            if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
                g_last_on_use_exit_ms[CHx] = now_ms;

            PID_speed.clear();
            PID_pressure.clear();
            pwm_zeroed = 1;
            x_prev[CHx] = 0;
            motion = filament_motion_enum::filament_motion_stop;
            Motion_control_set_PWM(CHx, 0);
            return;
        }

        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use && g_on_use_low_latch[CHx])
        {
            g_on_use_hi_pwm_us[CHx] = 0u;
            PID_speed.clear();
            PID_pressure.clear();
            pwm_zeroed = 1;
            x_prev[CHx] = 0;
            Motion_control_set_PWM(CHx, 0);
            return;
        }

        /* 【定点化】speed_set / now_speed 单位 µm/s；x / dm_autoload_x 单位 毫 PWM */
        int32_t speed_set = 0;
        const int32_t now_speed = speed_as5600[CHx];
        int32_t x = 0;
        /* PID 积分的时间步长，以 100 µs 为单位（除法为常量除法） */
        const uint32_t dt100 = time_us / 100u;

        // v4.0-tpu: TPU 通道识别结果(函数级) 供末尾间歇送料门控统一使用
        const _tpu_param *tpu_p_run = nullptr;

#if BMCU_DM_TWO_MICROSWITCH
        bool  dm_autoload_active = false;
        int32_t dm_autoload_x    = 0;
#endif

        // info o ostatnim wyjściu z on_use
        const uint64_t t_exit  = g_last_on_use_exit_ms[CHx];
        const bool had_on_use  = (t_exit != 0);
        const bool has_exit_ts = (t_exit > 1);
        uint64_t dt_exit = 0;
        if (has_exit_ts) dt_exit = (now_ms - t_exit);

        // aktywne tylko: idle + brak filamentu + kanał wpięty + kiedykolwiek był w on_use
        const bool post_on_use_active =
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) &&
            (MC_ONLINE_key_stu[CHx] == 0) &&
            filament_channel_inserted[CHx] &&
            had_on_use;

        const bool post_on_use_10s  = post_on_use_active && has_exit_ts && (dt_exit < 10000ull);

        const bool on_use_like =
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) ||
            (motion == filament_motion_enum::filament_motion_before_on_use) ||
            (motion == filament_motion_enum::filament_motion_stop_on_use) ||
            post_on_use_10s ||
            ((motion == filament_motion_enum::filament_motion_send) && send_stop_latch);

        bool  on_use_need_move = false;
        int32_t on_use_abs_err = 0;      /* 单位：厘百分比 */
        bool  on_use_linear    = false;

        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle)
        {
        #if BMCU_DM_TWO_MICROSWITCH
                    // --- DM autoload (Stage1 + Stage2) ---
                    if (filament_channel_inserted[CHx] && (dm_loaded[CHx] == 0u))
                    {
                        const uint8_t ks = MC_ONLINE_key_stu[CHx];
                        const uint32_t cur_count = encoder_count(CHx);

                        if (dm_fail_latch[CHx] &&
                            dm_auto_state[CHx] != DM_AUTO_S1_FAIL_RETRACT &&
                            dm_auto_state[CHx] != DM_AUTO_S2_FAIL_RETRACT &&
                            dm_auto_state[CHx] != DM_AUTO_S2_FAIL_EXTRA)
                        {
                            dm_autoload_active = true;
                            dm_autoload_x = 0;
                            MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);
                        }
                        else
                        {
                            if (dm_auto_state[CHx] == DM_AUTO_IDLE && dm_autoload_gate[CHx] == 0u)
                            {
                                if (ks == 2u)
                                {
                                    if (dm_autoload_gate[CHx] == 0u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                }
                                else if (ks == 1u)
                                {
                                    dm_autoload_gate[CHx] = 1u;
                                    dm_auto_state[CHx]    = DM_AUTO_S2_PUSH;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_count[CHx] = DM_AUTO_S2_TARGET_COUNT;
                                    dm_auto_last_count[CHx]   = cur_count;
                                }
                            }

                            switch (dm_auto_state[CHx])
                            {
                            case DM_AUTO_S1_DEBOUNCE:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks == 1u)
                                {
                                    dm_autoload_gate[CHx] = 1u;
                                    dm_auto_state[CHx] = DM_AUTO_S2_PUSH;
                                    dm_auto_try[CHx] = 0u;
                                    dm_auto_remain_count[CHx] = DM_AUTO_S2_TARGET_COUNT;
                                    dm_auto_last_count[CHx] = cur_count;
                                }
                                else if (ks != 2u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_DEBOUNCE_MS)
                                {
                                    dm_autoload_gate[CHx] = 1u;
                                    dm_auto_state[CHx] = DM_AUTO_S1_PUSH;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                break;

                            case DM_AUTO_S1_PUSH:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if (ks == 1u)
                                {
                                    dm_autoload_gate[CHx] = 1u;
                                    dm_auto_state[CHx]    = DM_AUTO_S2_PUSH;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_count[CHx] = DM_AUTO_S2_TARGET_COUNT;
                                    dm_auto_last_count[CHx]   = cur_count;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_TIMEOUT_MS)
                                {
                                    dm_fail_latch[CHx] = 1u;
                                    dm_auto_state[CHx] = DM_AUTO_S1_FAIL_RETRACT;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                else
                                {
                                    dm_autoload_x = -dir * DM_AUTO_PWM_PUSH;
                                }
                                break;

                            case DM_AUTO_S1_FAIL_RETRACT:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_FAIL_RETRACT_MS)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_S2_PUSH:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks != 1u)
                                {
                                    if (ks == 2u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                    else
                                    {
                                        dm_auto_reset_state(CHx);
                                    }
                                    break;
                                }

                                // remain -= moved
                                {
                                    const uint32_t moved = count_distance(cur_count, dm_auto_last_count[CHx]);
                                    dm_auto_last_count[CHx] = cur_count;
                                    const uint32_t remain = dm_auto_remain_count[CHx];
                                    dm_auto_remain_count[CHx] = moved < remain ? remain - moved : 0u;
                                }

                                if (MC_PULL_pct_cc[CHx] > DM_AUTO_BUF_ABORT_CC)
                                {
                                    uint8_t t = dm_auto_try[CHx];
                                    if (t < 255u) t++;
                                    dm_auto_try[CHx] = t;

                                    dm_auto_last_count[CHx] = cur_count;

                                    if (t >= 3u)
                                    {
                                        dm_fail_latch[CHx] = 1u;
                                        dm_auto_state[CHx] = DM_AUTO_S2_FAIL_RETRACT;
                                        dm_auto_remain_count[CHx] = DM_AUTO_S2_TARGET_COUNT;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                    else
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S2_RETRACT;
                                    }

                                    MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);
                                }
                                else if (dm_auto_remain_count[CHx] == 0u)
                                {
                                    dm_loaded[CHx] = 1u;

                                    dm_auto_reset_state(CHx);

                                    MC_STU_RGB_set(CHx, 0x38, 0x35, 0x32);
                                    dm_autoload_x = 0;
                                }
                                else
                                {
                                    dm_autoload_x = -dir * DM_AUTO_PWM_PUSH;
                                }
                                break;

                            case DM_AUTO_S2_RETRACT:
                                dm_autoload_active = true;

                                if (ks == 0u)
                                {
                                    dm_auto_reset_state(CHx);
                                    break;
                                }

                                // remain += moved
                                {
                                    const uint32_t moved = count_distance(cur_count, dm_auto_last_count[CHx]);
                                    dm_auto_last_count[CHx] = cur_count;
                                    const uint32_t remain = dm_auto_remain_count[CHx];
                                    dm_auto_remain_count[CHx] = moved < DM_AUTO_S2_TARGET_COUNT - remain ?
                                        remain + moved : DM_AUTO_S2_TARGET_COUNT;
                                }

                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if ((MC_PULL_pct_cc[CHx] <= DM_AUTO_BUF_RECOVER_CC) || (ks == 2u))
                                {
                                    dm_auto_last_count[CHx] = cur_count;

                                    if (ks == 1u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S2_PUSH;
                                    }
                                    else if (ks == 2u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                    else
                                    {
                                        dm_auto_reset_state(CHx);
                                    }
                                    dm_autoload_x = 0;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_S2_FAIL_RETRACT:
                            case DM_AUTO_S2_FAIL_EXTRA:
                            {
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                const uint32_t moved = count_distance(cur_count, dm_auto_last_count[CHx]);
                                dm_auto_last_count[CHx] = cur_count;
                                const uint32_t remain = dm_auto_remain_count[CHx];
                                dm_auto_remain_count[CHx] = moved < remain ? remain - moved : 0u;
                                const bool extra = dm_auto_state[CHx] == DM_AUTO_S2_FAIL_EXTRA;
                                const uint64_t limit_ms = extra ? DM_AUTO_FAIL_EXTRA_MS : DM_AUTO_S1_FAIL_RETRACT_MS;

                                if (ks == 0u || dm_auto_remain_count[CHx] == 0u ||
                                    (now_ms - dm_auto_t0_ms[CHx]) >= limit_ms)
                                {
                                    dm_auto_reset_state(CHx);
                                }
                                else if (!extra && ks == 2u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_S2_FAIL_EXTRA;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;
                            }

                            default:
                                dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                dm_auto_try[CHx]      = 0u;
                                dm_auto_remain_count[CHx] = 0u;
                                dm_auto_t0_ms[CHx]    = 0ull;
                                break;
                            }
                        }
                    }

                    if (dm_autoload_active)
                    {
                        x = dm_autoload_x;
                        PID_pressure.clear();
                        PID_speed.clear();
                    }
                    else
        #endif

            if (MC_ONLINE_key_stu[CHx] == 0)
            {
                if (!filament_channel_inserted[CHx] || !had_on_use)
                {
                    PID_pressure.clear();
                    pwm_zeroed = 1;
                    x_prev[CHx] = 0;
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }

                if (post_on_use_10s)
                {
                    if ((uint8_t)MC_PULL_pct[CHx] >= 49u)
                    {
                        x = 0;
                        PID_pressure.clear();
                        on_use_need_move = false;
                        on_use_abs_err = 0;
                    }
                    else
                    {
                        const int32_t pct_cc = MC_PULL_pct_cc[CHx];
                        const int32_t err = pct_cc - 4900;    /* 49.0 % */

                        on_use_need_move = true;
                        on_use_abs_err   = -err;

                        x = dir * PID_pressure.caculate(err, dt100);

                        /* 输出上限 = 500 + 80×|误差%| (PWM)
                         * 定点：500000 + 800 × |误差厘%| 毫 PWM（80 PWM/% = 800 毫PWM/厘%） */
                        int32_t lim_x = 500 * kPwmM + 800 * on_use_abs_err;
                        if (lim_x > 900 * kPwmM) lim_x = 900 * kPwmM;

                        if (x >  lim_x) x =  lim_x;
                        if (x < -lim_x) x = -lim_x;
                        if (x * dir > 0)
                        {
                            x = 0;
                            PID_pressure.clear();
                            on_use_need_move = false;
                            on_use_abs_err   = 0;
                        }
                    }
                }
                else
                {
                    // po 10s: idle jakby filament był -> tylko na krańcach (MC_PULL_stu != 0)
                    if (MC_PULL_stu[CHx] != 0)
                    {
                        const int32_t pct_cc = MC_PULL_pct_cc[CHx];
                        x = dir * PID_pressure.caculate(pct_cc - 5000, dt100);
                    }
                    else
                    {
                        x = 0;
                        PID_pressure.clear();
                    }
                }
            }
            else
            {
                // normalny idle z filamentem
                if (MC_PULL_stu[CHx] != 0)
                {
                    const int32_t pct_cc = MC_PULL_pct_cc[CHx];
                    x = dir * PID_pressure.caculate(pct_cc - 5000, dt100);
                }
                else
                {
                    x = 0;
                    PID_pressure.clear();
                }
            }
        }
        else if (motion == filament_motion_enum::filament_motion_redetect) // wyjście do braku filamentu -> ponowne podanie
        {
            x = -dir * (900 * kPwmM);
        }
        else if (MC_ONLINE_key_stu[CHx] != 0) // kanał aktywny i jest filament
        {
            if (motion == filament_motion_enum::filament_motion_before_pull_back)
            {
                const int32_t pct_cc = MC_PULL_pct_cc[CHx];
                constexpr int32_t target = 5000;      /* 50.0 % */

                const int32_t start_retract = target + 25;   /* +0.25 % = 25 厘% */
                const int32_t stop_retract  = target;

                static uint8_t pb_active[4] = {0,0,0,0};

                pb_active[CHx] = hyst_u8(pb_active[CHx], pct_cc, start_retract, stop_retract);

                if (!pb_active[CHx])
                {
                    x = 0;
                    on_use_need_move = false;
                    on_use_abs_err   = 0;
                }
                else
                {
                    const int32_t err = pct_cc - target; // dodatni
                    on_use_need_move = true;
                    on_use_abs_err   = err;

                    const int32_t mag = retract_mag_from_err(err, 850 * kPwmM);

                    x = dir * mag;          // tylko cofanie
                    if (x * dir < 0) x = 0;
                }
            }
            else if (motion == filament_motion_enum::filament_motion_before_on_use)
            {
                const int32_t pct_cc = MC_PULL_pct_cc[CHx];

                hold_load(
                    pct_cc,
                    dir,
                    PID_pressure,
                    post_sendout_retract_thresh_pct,
                    retract_hys_active,
                    x,
                    on_use_need_move,
                    on_use_abs_err,
                    on_use_linear
                );
            }
            else if (motion == filament_motion_enum::filament_motion_stop_on_use)
            {
                PID_pressure.clear();
                pwm_zeroed = 1;
                x_prev[CHx] = 0;
                Motion_control_set_PWM(CHx, 0);
                return;
            }
            else if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
            {
                const int32_t pct_cc = MC_PULL_pct_cc[CHx];

                // v4.0-tpu: 运行时识别 TPU 通道，使用对应型号参数替代刚性常量
                const _tpu_param *tpu_p = nullptr;
                tpu_p_run = nullptr;
                if (ams[motion_control_ams_num].filament[CHx].filament_type == _filament_type::tpu)
                {
                    tpu_p = tpu_param_fixed((uint8_t)CHx);
                    tpu_p_run = tpu_p;
                }

                /* 【定点化】target_pct / band_hi / pct_fast_onuse 单位为厘百分比；
                 * pwm_* 单位为毫 PWM。TPU 参数表字段已同步改为整数。 */
                int32_t target_pct = tpu_p ? tpu_p->on_use_target_pct : MC_ON_USE_TARGET_PCT;
                int32_t band_hi    = tpu_p ? tpu_p->on_use_band_hi    : MC_ON_USE_BAND_HI_PCT;

                bool handoff_active = false;
                if (on_use_handoff_ceiling_pct >= 0)
                {
                    const bool timed_out =
                        (on_use_handoff_t0_ms == 0ull) ||
                        ((now_ms - on_use_handoff_t0_ms) >= ON_USE_HANDOFF_MS);
                    const bool reached_target = pct_cc <= target_pct;

                    if (timed_out || reached_target)
                    {
                        on_use_handoff_ceiling_pct = -100;
                        on_use_handoff_t0_ms = 0ull;
                    }
                    else
                    {
                        handoff_active = true;
                    }
                }

                const int32_t band_hi_eff = handoff_active
                    ? (on_use_handoff_ceiling_pct + ON_USE_HANDOFF_CEILING_HYS_CC)
                    : band_hi;

                constexpr int32_t pwm_lo = 380 * kPwmM;
                // v4.0-tpu: 常规 on_use 推力上限按 TPU 型号降级（软料防过推/啃料）
                const int32_t pct_fast_onuse = tpu_p ? (target_pct + 500) : 5000;   /* +5 % = 500 厘% */
                const int32_t pwm_fast_onuse = tpu_p ? tpu_p->feed_pwm_hi : 900 * kPwmM;
                const int32_t pwm_cap        = tpu_p ? tpu_p->feed_pwm_hi : 900 * kPwmM;

                /* slope 单位：毫 PWM / 厘%（分子分母同量纲，直接相除） */
                const int32_t slope =
                    (pwm_fast_onuse - pwm_lo) / ((target_pct - MC_ON_USE_BAND_LO_DELTA) - pct_fast_onuse);

                retract_hys_active = 0;

                if (pct_cc >= (target_pct - MC_ON_USE_BAND_LO_DELTA) && pct_cc <= band_hi_eff)
                {
                    x = 0;
                    PID_pressure.clear();
                    on_use_need_move = false;
                    on_use_abs_err   = 0;
                }
                else if (pct_cc < (target_pct - MC_ON_USE_BAND_LO_DELTA))
                {
                    const int32_t err = pct_cc - target_pct;
                    on_use_need_move = true;
                    on_use_abs_err   = -err;

                    int32_t pwm;
                    if (pct_cc >= pct_fast_onuse)
                        pwm = pwm_lo + ((target_pct - MC_ON_USE_BAND_LO_DELTA) - pct_cc) * slope;
                    else
                        pwm = pwm_fast_onuse + (pct_fast_onuse - pct_cc) * slope;

                    if (pwm > pwm_cap) pwm = pwm_cap;

                    x = -dir * pwm;
                    PID_pressure.clear();
                    on_use_linear = true;
                }
                else
                {
                    on_use_need_move = true;

                    const int32_t control_target = handoff_active
                        ? on_use_handoff_ceiling_pct : target_pct;
                    const int32_t err = pct_cc - control_target;
                    on_use_abs_err = fx_abs(err);

                    x = dir * PID_pressure.caculate(err, dt100);

                    int32_t lim_x = 500 * kPwmM + 800 * on_use_abs_err;
                    if (lim_x > 900 * kPwmM) lim_x = 900 * kPwmM;

                    if (x >  lim_x) x =  lim_x;
                    if (x < -lim_x) x = -lim_x;

                    constexpr int32_t retrig = 5500;    /* 55.0 % */
                    if (!handoff_active && err > 0 && pct_cc >= retrig)
                    {
                        /* mul = 1 + 0.5×(pct-55)，Q8 定点（256 = 1.0） */
                        int32_t mul_q8 = 256 + ((pct_cc - retrig) * 256) / 200;
                        if (mul_q8 > 768) mul_q8 = 768;         /* 上限 3.0 */
                        x = (x * mul_q8) / 256;
                        if (x >  950 * kPwmM) x =  950 * kPwmM;
                        if (x < -950 * kPwmM) x = -950 * kPwmM;
                    }
                }
            }
            else
            {
                if (motion == filament_motion_enum::filament_motion_stop)
                {
                    PID_speed.clear();
                    pwm_zeroed = 1;
                    x_prev[CHx] = 0;
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }

                bool do_speed_pid = true;

                if (motion == filament_motion_enum::filament_motion_send)
                {
                    const int32_t pct_cc = MC_PULL_pct_cc[CHx];

                    if (!send_len_abort)
                    {
                        constexpr uint32_t SEND_MAX_COUNT = distance_counts(10 * kUmPerM);
                        if (count_distance(encoder_count(CHx), send_start_count) >= SEND_MAX_COUNT)
                            send_len_abort = 1;
                    }

                    if (send_len_abort)
                    {
                        PID_speed.clear();
                        PID_pressure.clear();
                        pwm_zeroed = 1;
                        x_prev[CHx] = 0;
                        Motion_control_set_PWM(CHx, 0);
                        return;
                    }

                    // HARD STOP（阈值原为整数百分比，此处换算到厘百分比）
                    if (pct_cc >= MC_LOAD_S1_HARD_STOP_PCT * kPctC)
                    {
                        send_hard = true;
                        PID_speed.clear();
                        PID_pressure.clear();
                        pwm_zeroed = 1;
                        x_prev[CHx] = 0;
                        Motion_control_set_PWM(CHx, 0);
                        return;
                    }

                    if (send_hard)
                    {
                        if (pct_cc >= (MC_LOAD_S1_HARD_STOP_PCT - MC_LOAD_S1_HARD_HYS) * kPctC)
                        {
                            PID_speed.clear();
                            PID_pressure.clear();
                            pwm_zeroed = 1;
                            x_prev[CHx] = 0;
                            Motion_control_set_PWM(CHx, 0);
                            return;
                        }
                        send_hard = false;
                    }

                    if (!send_stop_latch && (pct_cc >= MC_LOAD_S1_FAST_PCT * kPctC))
                    {
                        send_stop_latch = true;

                        int32_t p = pct_cc;
                        if (p < 0) p = 0;
                        if (p > 10000) p = 10000;

                        post_sendout_retract_thresh_pct = p;
                        retract_hys_active = 0;

                        PID_speed.clear();
                        PID_pressure.clear();
                    }

                    if (send_stop_latch)
                    {
                        do_speed_pid = false;

                        hold_load(
                            pct_cc,
                            dir,
                            PID_pressure,
                            post_sendout_retract_thresh_pct,
                            retract_hys_active,
                            x,
                            on_use_need_move,
                            on_use_abs_err,
                            on_use_linear
                        );
                    }
                    else
                    {
                        /* 软启动：300ms 内速度由 10 mm/s 线性升到 60 mm/s
                         * 【定点化】速度单位 µm/s，插值参数 t 用 Q8（256 = 1.0） */
                        constexpr uint32_t SEND_SOFTSTART_MS = 300u;
                        constexpr int32_t  V0 = 10 * 1000;   /* 10 mm/s → µm/s */
                        constexpr int32_t  V  = 60 * 1000;   /* 60 mm/s → µm/s */

                        const uint32_t dt = (send_start_ms != 0)
                            ? (uint32_t)(now_ms - send_start_ms) : 1000000u;

                        if (dt < SEND_SOFTSTART_MS)
                        {
                            int32_t t_q8 = (int32_t)((dt * 256u) / SEND_SOFTSTART_MS);
                            if (t_q8 < 0) t_q8 = 0;
                            if (t_q8 > 256) t_q8 = 256;
                            speed_set = V0 + ((V - V0) * t_q8) / 256;
                        }
                        else
                        {
                            speed_set = V;
                        }
                    }
                }

                if (motion == filament_motion_enum::filament_motion_pull) // cofanie
                {
                    speed_set = g_pull_speed_set[CHx]; // dynamiczne (liniowo w końcówce)
                }

                if (do_speed_pid)
                    x = dir * PID_speed.caculate(now_speed - speed_set, dt100);
            }
        }
        else
        {
            x = 0;
        }

        // stałe tryby
        const bool pull_mode = (motion == filament_motion_enum::filament_motion_pull);
        const bool pb_mode = (motion == filament_motion_enum::filament_motion_before_pull_back);

        const bool send_stop_hold_mode =
            (motion == filament_motion_enum::filament_motion_send) && send_stop_latch;

        const bool hold_mode =
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) ||
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) ||
            (motion == filament_motion_enum::filament_motion_before_on_use) ||
            (motion == filament_motion_enum::filament_motion_stop_on_use) ||
            post_on_use_active ||
            send_stop_hold_mode;

        const int deadband =
            pb_mode ? 0 :
            (hold_mode ? 1 : (pull_mode ? 2 : 10));

        int32_t pwm0 =
            pb_mode ? 0 :
            (hold_mode ? 420 * kPwmM : pwm_zero);

        if (pull_mode)
        {
            /* 减速系数 k = 剩余距离 / 15mm，Q8 定点（256 = 1.0） */
            int32_t k_q8 = (int32_t)((g_pull_remain_um[CHx] * 256) / PULL_RAMP_UM);
            k_q8 = fx_clamp(k_q8, 0, 256);

            // daleko: ~pwm_zero (500), przy końcu: >=400
            pwm0 = PULL_PWM_MIN + ((pwm_zero - PULL_PWM_MIN) * k_q8) / 256;

            if (pwm0 < PULL_PWM_MIN) pwm0 = PULL_PWM_MIN;
        }

        if (x > deadband * kPwmM)
        {
            if (x < pwm0) x = pwm0;
        }
        else if (x < -deadband * kPwmM)
        {
            if (-x < pwm0) x = -pwm0;
        }
        else
        {
            x = 0;
        }

        // clamp
        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle)
        {
        #if BMCU_DM_TWO_MICROSWITCH
            const int32_t lim_x = dm_autoload_active ? DM_AUTO_IDLE_LIM : 800 * kPwmM;
            if (x >  lim_x) x =  lim_x;
            if (x < -lim_x) x = -lim_x;
        #else
            constexpr int32_t PWM_IDLE_LIM = 800 * kPwmM;
            if (x >  PWM_IDLE_LIM) x =  PWM_IDLE_LIM;
            if (x < -PWM_IDLE_LIM) x = -PWM_IDLE_LIM;
        #endif
        }
        else
        {
            if (x >  PWM_lim * kPwmM) x =  PWM_lim * kPwmM;
            if (x < -PWM_lim * kPwmM) x = -PWM_lim * kPwmM;
        }

        // ON_USE: min PWM + anty-stall
        /* 【定点化】stall_s 由"秒"改为"微秒"累加 *
         *              判定阈值 0.15s → 150000 µs，0.8s → 800000 µs */
        static int32_t  stall_s[4] = {0,0,0,0};
        static uint64_t block_until_ms[4] = {0,0,0,0};

        if (on_use_like)
        {
            if (now_ms < block_until_ms[CHx])
            {
                PID_pressure.clear();
                pwm_zeroed = 1;
                x_prev[CHx] = 0;
                Motion_control_set_PWM(CHx, 0);
                return;
            }

            if (on_use_need_move && x != 0)
            {
                if (!on_use_linear)
                {
                    /* 误差 >= 1.3 % 时，输出端至少给 500 PWM（= 500000 毫 PWM） */
                    const int MIN_MOVE_PWM = (on_use_abs_err >= 130) ? 500 : 0;
                    if (MIN_MOVE_PWM)
                    {
                        /* 毫 PWM → PWM（四舍五入后取绝对值） */
                        const int xi = (x >= 0) ? ((x + 500) / 1000) : ((x - 500) / 1000);
                        const int ax = (xi < 0) ? -xi : xi;
                        if (ax < MIN_MOVE_PWM)
                            x = (x > 0) ? (MIN_MOVE_PWM * kPwmM) : (-MIN_MOVE_PWM * kPwmM);
                    }
                }
            }

            const bool motor_not_moving = (fx_abs(now_speed) < 1000);   /* < 1 mm/s = 1000 µm/s */

            if (on_use_need_move && motor_not_moving && (on_use_abs_err >= 200) && (fx_abs(x) >= 450 * kPwmM))
            {
                stall_s[CHx] += (int32_t)time_us;

                if (stall_s[CHx] > 150000)          /* 0.15 s */
                {
                    constexpr int32_t KICK_PWM = 850 * kPwmM;
                    x = (x > 0) ? KICK_PWM : -KICK_PWM;
                }

                if (stall_s[CHx] > 800000)          /* 0.8 s */
                {
                    stall_s[CHx] = 0;
                    block_until_ms[CHx] = now_ms + 500;
                    PID_pressure.clear();
                    pwm_zeroed = 1;
                    x_prev[CHx] = 0;
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
            }
            else
            {
                stall_s[CHx] = 0;
            }
        }
        else
        {
            stall_s[CHx] = 0;
            block_until_ms[CHx] = 0ull;
        }

        if (motion == filament_motion_enum::filament_motion_redetect)
        {
            const int pwm_out = (int)x;
            pwm_zeroed = (pwm_out == 0);
            x_prev[CHx] = x;
            Motion_control_set_PWM(CHx, pwm_out);
            return;
        }

        const bool use_ramping =
            ((motion == filament_motion_enum::filament_motion_send) && !send_stop_latch) ||
            (motion == filament_motion_enum::filament_motion_pull);

        if (use_ramping)
        {
            const bool pull_soft_start =
                (motion == filament_motion_enum::filament_motion_pull) &&
                (pull_start_ms != 0) &&
                ((now_ms - pull_start_ms) < 400ull);

            /* 【定点化】斜率原为"PWM/秒"，现换算为"毫 PWM/µs"的 Q4 定点（16 = 1.0）：
             *   rate_q4 = rate[PWM/s] × 1000[毫PWM/PWM] × 16 / 1e6[µs/s]
             *           = rate × 16 / 1000
             *   4500 → 72、6500 → 104、2500 → 40、25000 → 400、18000 → 288（均为整数）
             * max_step = (rate_q4 × dt_µs) >> 4，最大 400×200000 = 8e7 < 2^31 ✓ */
            int32_t rate_up_q4   = 72;      /* 4500 PWM/s */
            int32_t rate_down_q4 = 104;     /* 6500 PWM/s */

            if (pull_soft_start) rate_up_q4 = 40;   /* 2500 PWM/s */

            if (motion == filament_motion_enum::filament_motion_send)
            {
                rate_down_q4 = 400;         /* 25000 PWM/s */
                rate_up_q4   = 288;         /* 18000 PWM/s */
            }

            const int32_t max_step_up   = (rate_up_q4   * (int32_t)time_us) >> 4;
            const int32_t max_step_down = (rate_down_q4 * (int32_t)time_us) >> 4;

            const int32_t prev = x_prev[CHx];
            const int32_t lo = prev - max_step_down;
            const int32_t hi = prev + max_step_up;

            if (x < lo) x = lo;
            if (x > hi) x = hi;
        }

        int pwm_out0 = x / 1000;    /* 毫 PWM → PWM（截断，与旧 (int)x 行为一致） */

        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use && !g_on_use_low_latch[CHx])
        {
            if (MC_ONLINE_key_stu[CHx] == 0u)
            {
                g_on_use_hi_pwm_us[CHx] = 0u;
            }
            else
            {
                const int32_t pct_cc = MC_PULL_pct_cc[CHx];

                if (pct_cc < 4000)      /* 40.0 % */
                {
                    g_on_use_low_latch[CHx] = 1u;
                    g_on_use_jam_latch[CHx] = 1u;
                }
                else
                {
                    const int pwm_cmd = pwm_out0;
                    const int ax = (pwm_cmd < 0) ? -pwm_cmd : pwm_cmd;

                    const bool push_hi =
                        (dir != 0) &&
                        ((pwm_cmd * dir) < 0) &&
                        (ax > 800);

                    if (push_hi)
                    {
                        /* 时间步长本身就是微秒，无需再乘 1e6 */
                        const uint32_t add_us = time_us;

                        uint32_t t1 = g_on_use_hi_pwm_us[CHx] + add_us;
                        if (t1 > 20000000u) t1 = 20000000u;
                        g_on_use_hi_pwm_us[CHx] = t1;

                        if (t1 >= 20000000u)
                        {
                            g_on_use_low_latch[CHx] = 1u;
                            g_on_use_jam_latch[CHx] = 0u;
                        }
                    }
                    else
                    {
                        g_on_use_hi_pwm_us[CHx] = 0u;
                    }
                }

                if (g_on_use_low_latch[CHx])
                {
                    g_on_use_hi_pwm_us[CHx] = 0u;

                    auto &A = ams[motion_control_ams_num];
                    if (g_on_use_jam_latch[CHx] && A.now_filament_num == (uint8_t)CHx)
                        A.pressure = 0xF06Fu;

                    MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                    PID_speed.clear();
                    PID_pressure.clear();
                    pwm_zeroed = 1;
                    x_prev[CHx] = 0;
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
            }
        }
        else
        {
            g_on_use_hi_pwm_us[CHx] = 0u;
        }

        // v4.0-tpu: TPU 间歇送料门控（统一出口，覆盖 Stage2 装填/on_use/避让所有分支）
        // 软料不能被持续推力顶着，否则料被压缩、从缓冲头间隙挤出、进不了管。
        // 周期内分"推窗口(push_on_ms, 正常给PWM)"和"停窗口(剩余时间, 强制 PWM=0)"，
        // 停窗口让电机停转、料松弛/被打印机拉走，下一推窗口再补。越软停越久。
        // 仅在 TPU 通道且为"往前送料"(x 与 dir 同向)时生效；回退/拉料不受影响。
        if (tpu_p_run != nullptr)
        {
            const uint16_t cyc = tpu_p_run->push_cycle_ms;
            const uint16_t on  = tpu_p_run->push_on_ms;
            if (cyc > 0u && on < cyc)
            {
                const uint32_t phase = (uint32_t)now_ms % cyc;
                const bool pushing_fwd = (dir != 0) && ((pwm_out0 * dir) > 0);
                if (pushing_fwd && phase >= (uint32_t)on)
                {
                    pwm_out0 = 0;   // 停窗口：电机停转，让软料松弛
                }
            }
        }

        const int pwm_out = pwm_out0;
        pwm_zeroed = (pwm_out == 0);
        x_prev[CHx] = x;
        Motion_control_set_PWM(CHx, pwm_out);
    }
};

/** @brief 4通道电机控制器实例数组 */
_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};
/** @brief 各通道上次PWM输出值（用于斜率限制） */
int32_t _MOTOR_CONTROL::x_prev[4] = {0,0,0,0};

/**
 * @brief 设置指定通道电机PWM输出
 *        正PWM=set1有效(正转), 负PWM=set2有效(反转), 0=制动(两侧高电平)
 *        CH3→TIM2, CH2→TIM3, CH1→TIM4(CH1/CH2), CH0→TIM4(CH3/CH4)
 */
void Motion_control_set_PWM(uint8_t CHx, int PWM)
{
    if (CHx >= kChCount) return;
    if (PWM && !MC_PULL_calibration_is_valid(CHx)) PWM = 0;
    uint16_t set1 = 0, set2 = 0;

    if (PWM > 0)       set1 = (uint16_t)PWM;
    else if (PWM < 0)  set2 = (uint16_t)(-PWM);
    else { set1 = 1000; set2 = 1000; }

    switch (CHx)
    {
    case 3:
        TIM_SetCompare1(TIM2, set1);
        TIM_SetCompare2(TIM2, set2);
        break;
    case 2:
        TIM_SetCompare1(TIM3, set1);
        TIM_SetCompare2(TIM3, set2);
        break;
    case 1:
        TIM_SetCompare1(TIM4, set1);
        TIM_SetCompare2(TIM4, set2);
        break;
    case 0:
        TIM_SetCompare3(TIM4, set1);
        TIM_SetCompare4(TIM4, set2);
        break;
    default:
        break;
    }
}

/* ==================== AS5600 距离/速度计算 ==================== */

/** @brief 各通道AS5600原始角度累计值（差分计算速度用） */
int32_t as5600_distance_save[4] = {0,0,0,0};

/**
 * @brief 更新AS5600编码器数据，计算各通道速度和累计位移
 *        执行频率：每毫秒一次（min_poll_ticks限流）
 *        功能：读取角度→差分计算→转换为mm和mm/s→更新传感器健康→累加位移
 */
void AS5600_distance_updata(uint32_t now_ticks)
{
    static uint32_t last_valid_ticks[4] = {0,0,0,0};
    static uint32_t last_poll_ticks = 0u;
    static uint8_t  was_ok[4] = {0,0,0,0};
    static uint32_t last_stu_ticks = 0u;

    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    uint32_t tpus = time_hw_tpus;
    if (!tpus) tpus = 1u;

    uint32_t min_poll_ticks = tpm;
    if ((uint32_t)(now_ticks - last_poll_ticks) < min_poll_ticks)
        return;

    last_poll_ticks = now_ticks;

    if ((uint32_t)(now_ticks - last_stu_ticks) >= (200u * tpm))
    {
        last_stu_ticks = now_ticks;
        MC_AS5600.updata_stu();
    }

    const uint8_t valid = MC_AS5600.updata_angle();
    const uint32_t sample_ticks = time_ticks32();
    uint32_t cached_dt_ticks = 0u;
    uint32_t cached_dt_us    = 0u;

    for (uint8_t i = 0; i < kChCount; i++)
    {
        const bool ok_now = (valid & (1u << i)) != 0u && (MC_AS5600.magnet_stu[i] != AS5600_soft_IIC_many::offline);

        if (ok_now)
        {
            g_as5600_fail[i] = 0;
            if (g_as5600_okstreak[i] < 255u) g_as5600_okstreak[i]++;
            if (g_as5600_okstreak[i] >= kAS5600_OK_RECOVER) g_as5600_good[i] = 1u;
        }
        else
        {
            g_as5600_okstreak[i] = 0u;
            if (g_as5600_fail[i] < 255u) g_as5600_fail[i]++;
            if (g_as5600_fail[i] >= kAS5600_FAIL_TRIP) g_as5600_good[i] = 0u;
        }

        if (!AS5600_is_good(i))
        {
            was_ok[i] = 0u;
            speed_as5600[i] = 0;
            continue;
        }

        if (!ok_now) continue;

        if (!was_ok[i])
        {
            as5600_distance_save[i] = MC_AS5600.raw_angle[i];
            speed_as5600[i] = 0;
            was_ok[i] = 1u;
            last_valid_ticks[i] = sample_ticks;
            continue;
        }

        const int32_t last = as5600_distance_save[i];
        const int32_t now  = MC_AS5600.raw_angle[i];

        int32_t diff = now - last;
        if (diff > 2048) diff -= 4096;
        if (diff < -2048) diff += 4096;

        const uint32_t dt_ticks = sample_ticks - last_valid_ticks[i];
        if (!dt_ticks) continue;
        last_valid_ticks[i] = sample_ticks;
        as5600_distance_save[i] = now;
        as5600_odometer_count[i] += diff;
        if (dt_ticks != cached_dt_ticks)
        {
            cached_dt_ticks = dt_ticks;
            /* 时间步长换算：dt_ticks / tpus = 微秒（tpus = 每微秒的 tick 数） */
            cached_dt_us = dt_ticks / tpus;
            if (!cached_dt_us) cached_dt_us = 1u;
        }

        /* 【定点化】速度 = 计数差 × 5752.0675 µm/count ÷ dt(µs)。
         * 计数差最大 ±2048，直接与系数相乘会溢出 int32 ⇒ 用 uint64 中间量；
         * 该除法复用已链接的 __udivdi3（来自 time_ms_fast_from_ticks64），不额外付费。 */
        const uint32_t adiff = (diff < 0) ? (uint32_t)(0 - diff) : (uint32_t)diff;
        const uint32_t amag  = (uint32_t)(((uint64_t)adiff * (uint32_t)(-kAS5600_SPEED_NUM)) / cached_dt_us);
        /* kAS5600_SPEED_NUM 为负（等价于上游负系数 kAS5600_MM_PER_CNT）：
         * 计数增加(diff>0) = 耗材被拉回 ⇒ 速度为负；反之送丝为正。 */
        speed_as5600[i] = (diff > 0) ? -(int32_t)amag : (int32_t)amag;
    }
}

/* ==================== 耗材运动状态枚举 ==================== */

/**
 * @brief 耗材当前物理位置状态
 *        idle→sending_out→using→before_pull_back→pulling_back→redetect→idle
 */
enum filament_now_position_enum
{
    filament_idle,                /**< 空闲，耗材在BMCU内 */
    filament_sending_out,         /**< 正在送丝 */
    filament_using,               /**< 打印中 */
    filament_before_pull_back,    /**< 回退前准备（释放压力） */
    filament_pulling_back,        /**< 正在回退 */
    filament_redetect,            /**< 重新检测（回退后推入确认） */
};

/** @brief 各通道的当前耗材位置状态 */
static filament_now_position_enum filament_now_position[4];
static uint32_t filament_pull_back_count[4];

#ifdef BMCU_RETRACT_UM_ANCHOR
/**
 * @brief 回抽长度锚点（单位 µm）——批量构建的二进制替换目标
 *
 * 占位值 123456000 µm = 123.456 m，是不可能出现的真实回抽长度，
 * 一旦忘了替换就会立刻暴露。build_fast.py 会把镜像里这 4 字节
 * 替换为 `round(AMS_RETRACT_LEN * 1e6)`。
 *
 * `volatile` 是必需的：它保证该对象拥有真实存储（.rodata），并且读取
 * 不会被 `-flto -Os` 折叠成立即数，从而在镜像中留下唯一锚点。
 * （若写成常量表达式，编译器会编成 `lui`+`addi` 立即数，
 *   4 字节锚点不会出现在镜像里，39 个回抽长度固件会全部退化成一个。）
 */
volatile const int32_t moc_pull_back_um_anchor = 123456000; /* 0x075BCA00 */
#endif

/** @brief 回抽目标步数（回抽长度经 distance_counts() 换算，见 MOTOR_init） */
static uint32_t kPullBackCount = 0;
/** @brief 各通道回抽目标步数（见 MOTOR_init） */
static uint32_t filament_pull_back_target[4] = {0, 0, 0, 0};

// BEFORE_PULLBACK: zapis realnie "wycofanej" drogi (counts) (sumowanie całego wycofania)
static uint32_t before_pb_last_count[4]      = {0,0,0,0};
static uint32_t before_pb_retracted_count[4] = {0,0,0,0};
static int8_t before_pb_sign[4]        = {0,0,0,0};

/* ===== redetect 确认状态（开关反抖）===== */
/** @brief 本轮 redetect 是否已初始化 */
static bool     rd_ready[4]   = {false, false, false, false};
/** @brief 开关连续有料起始时刻(ms) */
static uint64_t rd_hold_ms[4] = {0, 0, 0, 0};

/**
 * @brief 处理回退和重新检测运动序列
 *        回退阶段：动态速度拉回目标距离
 *        重新检测阶段：推入确认是否到位
 * @return true=仍有通道在执行, false=全部完成
 */
static bool motor_motion_filamnet_pull_back_to_online_key(uint64_t time_now)
{
    bool wait = false;
    auto &A = ams[motion_control_ams_num];

    for (uint8_t i = 0; i < kChCount; i++)
    {
        switch (filament_now_position[i])
        {
        case filament_pulling_back:
        {
            MC_STU_RGB_set_latch(i, 0xFFu, 0x00u, 0xFFu, time_now, 1u);

            const uint32_t target = filament_pull_back_target[i];
            const uint32_t d = count_distance(encoder_count(i), filament_pull_back_count[i]);

            if (target == 0u || d >= target)
            {
                g_pull_remain_um[i]  = 0;
                g_pull_speed_set[i] = -PULL_V_FAST;
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                // v4.0-tpu: TPU 软料回弹，固定长度回抽额外多退 pull_comp_um（仅该通道识别为 TPU 时）
                int32_t pull_target_um = motion_control_pull_back_um;
                if (A.filament[i].filament_type == _filament_type::tpu)
                {
                    const _tpu_param *tp = tpu_param_lookup(A.filament[i].bambubus_filament_id);
                    pull_target_um += tp->pull_comp_um;
                }
                filament_pull_back_target[i] = distance_counts(pull_target_um);
                rd_ready[i] = false;              // 复位 redetect 确认状态
                filament_now_position[i] = filament_redetect;
            }
            else if (g_key_empty[i] >= kKeyEmptyPublications)
            {
                g_pull_remain_um[i]  = 0;
                g_pull_speed_set[i] = -PULL_V_FAST;
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                filament_pull_back_target[i] = kPullBackCount;
                filament_now_position[i] = filament_redetect;
            }
            else
            {
                bool should_stop = false;

                if (MC_ONLINE_key_stu[i] == 0)
                {
                    should_stop = true;
                }
#if BMCU_DM_TWO_MICROSWITCH
                else if (MC_ONLINE_key_stu[i] == 2)
                {
                    should_stop = true;
                }
#endif

                if (should_stop)
                {
                    g_pull_remain_um[i]  = 0;
                    g_pull_speed_set[i] = -PULL_V_FAST;
                    MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                    // v4.0-tpu: TPU 软料回弹，固定长度回抽额外多退 pull_comp_um（仅该通道识别为 TPU 时）
                    int32_t pull_target_um = motion_control_pull_back_um;
                    if (A.filament[i].filament_type == _filament_type::tpu)
                    {
                        const _tpu_param *tp = tpu_param_lookup(A.filament[i].bambubus_filament_id);
                        pull_target_um += tp->pull_comp_um;
                    }
                    filament_pull_back_target[i] = distance_counts(pull_target_um);
                    rd_ready[i] = false;              // 复位 redetect 确认状态
                    filament_now_position[i] = filament_redetect;
                }
                else
                {
                    /* 【定点化】剩余距离（微米）= 剩余计数 × 5.7520675 µm/count。
                     * 拆成"千位 + 余数"两步乘，避免 rest_cnt×5752 溢出 int32。 */
                    const uint32_t rest_cnt = (target > d) ? (target - d) : 0u;
                    const int32_t  remain_um = (int32_t)(
                        ((rest_cnt / 1000u) * 5752u) + (((rest_cnt % 1000u) * 5752u) / 1000u));
                    g_pull_remain_um[i] = remain_um;

                    /* 减速插值：k = remain / 15 mm，Q8 定点（256 = 1.0）
                     * remain_um ≤ 2e6 ⇒ ×256 ≤ 5.12e8，安全落在 int32 内 */
                    int32_t k_q8 = (remain_um * 256) / PULL_RAMP_UM;
                    if (k_q8 < 0)   k_q8 = 0;
                    if (k_q8 > 256) k_q8 = 256;

                    /* v = V_END + (V_FAST - V_END) × k，单位 µm/s（负值 = 回退） */
                    const int32_t v = PULL_V_END + ((PULL_V_FAST - PULL_V_END) * k_q8) / 256;
                    g_pull_speed_set[i] = -v;

                    MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pull, 100, time_now);
                }
            }

            wait = true;
            break;
        }

        case filament_redetect:
        {
            MC_STU_RGB_set_latch(i, 0xFFu, 0xFFu, 0x00u, time_now, 0u);

            /* 本轮 redetect 首次进入：初始化 */
            if (!rd_ready[i])
            {
                rd_ready[i]   = true;
                rd_hold_ms[i] = 0ull;
            }

            /* 开关反抖：有料需连续保持 50ms 才算确认到位（time_now 单位为 ms） */
            bool key_on = (MC_ONLINE_key_stu[i] != 0);
#if BMCU_DM_TWO_MICROSWITCH
            if (MC_ONLINE_key_stu[i] == 2) key_on = false;   // 双微动：仅外部开关/缓冲手势不算到位
#endif
            if (key_on)
            {
                if (rd_hold_ms[i] == 0ull) rd_hold_ms[i] = time_now;
            }
            else
            {
                rd_hold_ms[i] = 0ull;
            }
            const bool key_hold_ok = (rd_hold_ms[i] != 0ull) &&
                                     (time_now - rd_hold_ms[i] >= 50u);

            if (key_hold_ok)
            {
                rd_ready[i] = false;
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                filament_now_position[i] = filament_idle;

                A.filament_use_flag = 0x00;
                A.filament[i].motion = _filament_motion::idle;
            }
            else
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_redetect, 100, time_now);
            }

            wait = true;
            break;
        }

        default:
            break;
        }
    }

    return wait;
}

/**
 * @brief 运动状态切换主函数
 *        根据打印机命令(A.filament[i].motion)切换内部状态机
 *        支持: before_on_use, stop_on_use, send_out, pull_back, on_use, idle等
 */
static void motor_motion_switch(uint64_t time_now)
{
    auto &A = ams[motion_control_ams_num];

    const uint8_t num = A.now_filament_num;
    const _filament_motion motion = (num < kChCount) ? A.filament[num].motion : _filament_motion::idle;

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (i != num)
        {
            filament_now_position[i] = filament_idle;

            if (filament_channel_inserted[i] && (MC_ONLINE_key_stu[i] != 0 || g_last_on_use_exit_ms[i] != 0))
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 1000, time_now);
            else
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 1000, time_now);

            continue;
        }

        if (num >= kChCount) continue;

        if (MC_ONLINE_key_stu[num] != 0)
        {
            switch (motion)
            {
            case _filament_motion::before_on_use:
            {
                filament_now_position[num] = filament_using;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_before_on_use, 300, time_now);
                MC_STU_RGB_set_latch(num, 0xFFu, 0xFFu, 0x00u, time_now, 0u);
                break;
            }

            case _filament_motion::stop_on_use:
            {
                filament_now_position[num] = filament_using;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_stop_on_use, 300, time_now);
                MC_STU_RGB_set_latch(num, 0xFFu, 0x00u, 0x00u, time_now, 0u);
                break;
            }

            case _filament_motion::send_out:
            {
                if (g_on_use_jam_latch[num])
                {
                    if (MC_PULL_pct_cc[num] > 8500)   /* 85.0 % */
                    {
                        g_on_use_low_latch[num] = 0u;
                        g_on_use_jam_latch[num] = 0u;
                        g_on_use_hi_pwm_us[num] = 0u;
                    }
                    else
                    {
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                        MC_STU_RGB_set_latch(num, 0x00u, 0xD5u, 0x2Au, time_now, 0u);
                        break;
                    }
                }

                MC_STU_RGB_set_latch(num, 0x00u, 0xD5u, 0x2Au, time_now, 0u);
                filament_now_position[num] = filament_sending_out;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_send, 100, time_now);
                break;
            }

            case _filament_motion::pull_back:
            {
                MC_STU_RGB_set_latch(num, 0xA0u, 0x2Du, 0xFFu, time_now, 1u);
                filament_now_position[num] = filament_pulling_back;

                filament_pull_back_count[num] = encoder_count(num);
                g_key_empty[num] = 0u;

                uint32_t target;
                if (g_on_use_jam_latch[num])
                {
                    target = distance_counts(100000);   /* 0.100 m = 100 mm = 100000 µm */
                }
                else
                {
                    const uint32_t already = before_pb_retracted_count[num];
                    target = already < kPullBackCount ? kPullBackCount - already : 0u;
                }

                filament_pull_back_target[num] = target;

                /* 【定点化】剩余距离（微米）：计数 → µm，拆两步避免溢出 */
                g_pull_remain_um[num] = (int32_t)(
                    ((target / 1000u) * 5752u) + (((target % 1000u) * 5752u) / 1000u));
                g_pull_speed_set[num] = -PULL_V_FAST;

                before_pb_retracted_count[num] = 0u;
                before_pb_sign[num]        = 0;
                before_pb_last_count[num]      = filament_pull_back_count[num];

                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pull, 100, time_now);
                break;
            }

            case _filament_motion::before_pull_back:
            {
                MC_STU_RGB_set_latch(num, 0xFFu, 0xA0u, 0x00u, time_now, 1u);

                if (filament_now_position[num] != filament_before_pull_back)
                {
                    filament_now_position[num] = filament_before_pull_back;
                    before_pb_last_count[num]      = encoder_count(num);
                    before_pb_retracted_count[num] = 0u;
                    before_pb_sign[num]        = 0;
                }

                {
                    const uint32_t count = encoder_count(num);
                    const int32_t dm = (int32_t)(before_pb_last_count[num] - count);
                    const uint32_t moved = count_distance(count, before_pb_last_count[num]);
                    before_pb_last_count[num] = count;

                    const int32_t pct_cc = MC_PULL_pct_cc[num];
                    const bool want_retract = (pct_cc > 5025);   /* 50.25 % */

                    if (want_retract)
                    {
                        if (before_pb_sign[num] == 0 && moved >= distance_counts(500))   /* 0.5 mm */
                            before_pb_sign[num] = (dm >= 0) ? 1 : -1;

                        if (before_pb_sign[num] > 0) {
                            if (dm > 0) before_pb_retracted_count[num] += moved;
                        } else if (before_pb_sign[num] < 0) {
                            if (dm < 0) before_pb_retracted_count[num] += moved;
                        }
                    }

                    if (before_pb_retracted_count[num] > distance_counts(2 * 1000000))   /* 2 m */
                        before_pb_retracted_count[num] = distance_counts(2 * 1000000);
                }

                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_before_pull_back, 300, time_now);
                break;
            }

            case _filament_motion::on_use:
            {
                filament_now_position[num] = filament_using;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_on_use, 300, time_now);
                MC_STU_RGB_set_latch(num, 0x00u, 0xB0u, 0xFFu, time_now, 0u);
                break;
            }

            case _filament_motion::idle:
            default:
            {
                filament_now_position[num] = filament_idle;

                if (g_on_use_jam_latch[num])
                {
                    MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                    MC_STU_RGB_set_latch(num, 0x38u, 0x35u, 0x32u, time_now, 0u);
                    break;
                }

                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100, time_now);

#if BMCU_DM_TWO_MICROSWITCH
                if (dm_fail_latch[num])      MC_STU_RGB_set_latch(num, 0xFFu, 0x00u, 0x00u, time_now, 0u);
                else if (dm_loaded[num])     MC_STU_RGB_set_latch(num, 0x38u, 0x35u, 0x32u, time_now, 0u);
                else                         MC_STU_RGB_set_latch(num, 0x00u, 0x00u, 0x00u, time_now, 0u);
#else
                MC_STU_RGB_set_latch(num, 0x38u, 0x35u, 0x32u, time_now, 0u);
#endif
                break;
            }
            }
        }
        else
        {
            filament_now_position[num] = filament_idle;
            MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100, time_now);
            MC_STU_RGB_set_latch(num, 0x00u, 0x00u, 0x00u, time_now, 0u);
        }
    }
}

/** @brief 应用基线状态灯颜色（空闲时根据通道状态显示颜色） */
static inline void stu_apply_baseline(int error, uint64_t now_ms)
{
    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (g_on_use_low_latch[i])
        {
            MC_STU_RGB_set(i, 0xFFu, 0x00u, 0x00u);
            continue;
        }

#if BMCU_DM_TWO_MICROSWITCH
        if (dm_fail_latch[i])
        {
            MC_STU_RGB_set(i, 0xFFu, 0x00u, 0x00u);
            continue;
        }

        const bool ins_ok = error ? true : filament_channel_inserted[i];
        const bool show_loaded =
            (dm_loaded[i] != 0u) &&
            (MC_ONLINE_key_stu[i] != 0u) &&
            ins_ok;

        if (show_loaded) MC_STU_RGB_set_latch(i, 0x38u, 0x35u, 0x32u, now_ms, 0u);
        else             MC_STU_RGB_set_latch(i, 0x00u, 0x00u, 0x00u, now_ms, 0u);
#else
        if (error)
        {
            if (MC_ONLINE_key_stu[i] != 0) MC_STU_RGB_set_latch(i, 0x38u, 0x35u, 0x32u, now_ms, 0u);
            else                           MC_STU_RGB_set_latch(i, 0x00u, 0x00u, 0x00u, now_ms, 0u);
        }
        else
        {
            if (MC_ONLINE_key_stu[i] != 0 && filament_channel_inserted[i])
                MC_STU_RGB_set_latch(i, 0x38u, 0x35u, 0x32u, now_ms, 0u);
            else
                MC_STU_RGB_set_latch(i, 0x00u, 0x00u, 0x00u, now_ms, 0u);
        }
#endif
    }
}


/**
 * @brief 电机运动主运行函数
 *        处理DM自动加载、时间步长计算、状态切换、PID控制、PWM输出、自动卸料、LED
 */
static void motor_motion_run(int error, uint64_t time_now, uint32_t now_ticks)
{
    for (uint8_t ch = 0; ch < kChCount; ch++)
    {
        if (MC_ONLINE_key_stu[ch] != 0u || !ADC_DMA_ready() ||
            filament_now_position[ch] != filament_pulling_back)
            g_key_empty[ch] = 0u;
        else if (g_key_sample_fresh && g_key_empty[ch] < kKeyEmptyPublications)
            ++g_key_empty[ch];
    }
#if BMCU_DM_TWO_MICROSWITCH
    for (uint8_t ch = 0; ch < kChCount; ch++)
    {
        if (!filament_channel_inserted[ch])
        {
            dm_loaded[ch]            = 1u;
            dm_fail_latch[ch]        = 0u;
            dm_auto_fields_reset(ch);
            dm_loaded_drop_t0_ms[ch] = 0ull;
            dm_autoload_gate[ch]     = 1u;
            dm_idle_empty[ch] = 0u;
            continue;
        }

        const uint8_t ks = MC_ONLINE_key_stu[ch];

        const bool idle = filament_now_position[ch] == filament_idle &&
            ams[motion_control_ams_num].filament[ch].motion == _filament_motion::idle &&
            !auto_unload_active[ch];
        if (!idle) dm_autoload_gate[ch] = 1u;
        if (!idle || ks != 0u || !ADC_DMA_ready()) dm_idle_empty[ch] = 0u;
        else if (g_key_sample_fresh && dm_idle_empty[ch] < kKeyEmptyPublications)
            ++dm_idle_empty[ch];

        if (ks == 0u)
        {
            if (dm_idle_empty[ch] >= kKeyEmptyPublications)
            {
                dm_autoload_gate[ch] = 0u;
                dm_fail_latch[ch] = 0u;
            }

            dm_loaded[ch]            = 0u;
            dm_fail_latch[ch]        = 0u;
            dm_auto_fields_reset(ch);
            dm_loaded_drop_t0_ms[ch] = 0ull;
            continue;
        }

        const auto &A = ams[motion_control_ams_num];
        const _filament_motion host_motion = A.filament[ch].motion;
        if (A.now_filament_num == ch && ks == 1u && g_key_sample_fresh &&
            !dm_fail_latch[ch] &&
            (host_motion == _filament_motion::before_on_use ||
             host_motion == _filament_motion::on_use ||
             host_motion == _filament_motion::stop_on_use))
        {
            dm_loaded[ch] = 1u;
            dm_autoload_gate[ch] = 1u;
            dm_auto_state[ch] = DM_AUTO_IDLE;
            dm_auto_try[ch] = 0u;
            dm_auto_remain_count[ch] = 0u;
            dm_auto_t0_ms[ch] = 0ull;
        }

        if (dm_loaded[ch] && (ks != 1u))
        {
            uint64_t t0 = dm_loaded_drop_t0_ms[ch];
            if (t0 == 0ull) dm_loaded_drop_t0_ms[ch] = time_now;
            else if ((time_now - t0) >= 100ull)
            {
                dm_loaded[ch]            = 0u;
                dm_loaded_drop_t0_ms[ch] = 0ull;

                dm_auto_fields_reset(ch);
            }
        }
        else
        {
            dm_loaded_drop_t0_ms[ch] = 0ull;
        }
    }
#endif

    static uint32_t last_ticks = 0u;
    static uint8_t  have_last_ticks = 0u;

    uint32_t dt_ticks = 0u;
    if (!have_last_ticks)
    {
        have_last_ticks = 1u;
    }
    else
    {
        dt_ticks = (uint32_t)(now_ticks - last_ticks);
    }
    last_ticks = now_ticks;

    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    const uint32_t max_dt_ticks = 200u * tpm;
    if (dt_ticks > max_dt_ticks) dt_ticks = max_dt_ticks;

    uint32_t tpus = time_hw_tpus;
    if (!tpus) tpus = 1u;

    const bool have_time_step = (dt_ticks != 0u);
    const uint32_t time_us = have_time_step ? (uint32_t)(dt_ticks / tpus) : 0u;   /* 微秒 */

    stu_apply_baseline(error, time_now);

#if BMCU_ONLINE_LED_FILAMENT_RGB
    auto &Acol = ams[motion_control_ams_num];
#endif

    const bool all_empty = all_no_filament();

    if (!error)
    {
        if (!motor_motion_filamnet_pull_back_to_online_key(time_now))
            motor_motion_switch(time_now);
    }
    else
    {
        for (uint8_t i = 0; i < kChCount; i++)
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
    }

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (!AS5600_is_good(i))
        {
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
            Motion_control_set_PWM(i, 0);
            continue;
        }

        if (!filament_channel_inserted[i] ||
#if BMCU_DM_TWO_MICROSWITCH
            dm_fail_latch[i] ||
#endif
            (!auto_unload_active[i] && MOTOR_CONTROL[i].motion != filament_motion_enum::filament_motion_pressure_ctrl_idle))
        {
            auto_unload_arm[i]          = 0u;
            auto_unload_active[i]       = 0u;
            auto_unload_blocked[i]      = 0u;
            auto_unload_arm_t0_ms[i]    = 0ull;
            auto_unload_active_t0_ms[i] = 0ull;
            auto_unload_empty_t0_ms[i]  = 0ull;
        }
        else
        {
            const int32_t pct = MC_PULL_pct_cc[i];
            const uint8_t ks = MC_ONLINE_key_stu[i];

            if (pct >= AUTO_UNLOAD_START_CC)
            {
                auto_unload_blocked[i] = 0u;

                if (!auto_unload_arm[i] && !auto_unload_active[i])
                {
                    auto_unload_arm[i] = 1u;
                    auto_unload_arm_t0_ms[i] = time_now;
                }
            }

            if (auto_unload_arm[i] && !auto_unload_active[i])
            {
                const uint64_t dt = time_now - auto_unload_arm_t0_ms[i];

                if ((pct > AUTO_UNLOAD_NEUTRAL_LO_CC) && (pct < AUTO_UNLOAD_NEUTRAL_HI_CC))
                {
                    if (!auto_unload_blocked[i] && dt <= AUTO_UNLOAD_ARM_MS)
                    {
                        auto_unload_active[i]       = 1u;
                        auto_unload_active_t0_ms[i] = time_now;
                        auto_unload_empty_t0_ms[i]  = 0ull;
                        auto_unload_blocked[i]      = 1u;
                    }

                    auto_unload_arm[i]       = 0u;
                    auto_unload_arm_t0_ms[i] = 0ull;
                }
                else if (dt > AUTO_UNLOAD_ARM_MS)
                {
                    auto_unload_arm[i]       = 0u;
                    auto_unload_arm_t0_ms[i] = 0ull;
                }
            }

            if (auto_unload_active[i])
            {
                if (pct < AUTO_UNLOAD_ABORT_CC)
                {
                    auto_unload_active[i]       = 0u;
                    auto_unload_active_t0_ms[i] = 0ull;
                    auto_unload_empty_t0_ms[i]  = 0ull;
                    auto_unload_blocked[i]      = 1u;
                }
                else if (ks == 1u)
                {
                    auto_unload_empty_t0_ms[i] = 0ull;

                    if ((time_now - auto_unload_active_t0_ms[i]) >= AUTO_UNLOAD_MAX_MS)
                    {
                        auto_unload_active[i]       = 0u;
                        auto_unload_active_t0_ms[i] = 0ull;
                        auto_unload_empty_t0_ms[i]  = 0ull;
                        auto_unload_blocked[i]      = 1u;
                    }
                }
                else
                {
                    if (auto_unload_empty_t0_ms[i] == 0ull)
                    {
                        auto_unload_empty_t0_ms[i] = time_now;
                    }
                    else if ((time_now - auto_unload_empty_t0_ms[i]) >= AUTO_UNLOAD_EMPTY_MS)
                    {
                        auto_unload_active[i]       = 0u;
                        auto_unload_active_t0_ms[i] = 0ull;
                        auto_unload_empty_t0_ms[i]  = 0ull;
                        auto_unload_blocked[i]      = 1u;
                    }
                }
            }
        }

        const bool manual_empty_pull =
#if BMCU_DM_TWO_MICROSWITCH
            !dm_fail_latch[i] &&
#endif
            filament_channel_inserted[i] &&
            (MC_ONLINE_key_stu[i] == 0u) &&
            (MC_PULL_pct_cc[i] > 8000) &&   /* 80 % */
            (auto_unload_active[i] == 0u);

        if (auto_unload_active[i])
        {
            int32_t x = MOTOR_CONTROL[i].dir * AUTO_UNLOAD_PWM_PULL;   /* 毫 PWM */
            if (x * MOTOR_CONTROL[i].dir < 0) x = 0;

            MOTOR_CONTROL[i].PID_speed.clear();
            MOTOR_CONTROL[i].PID_pressure.clear();
            MOTOR_CONTROL[i].pwm_zeroed = (x == 0) ? 1u : 0u;
            _MOTOR_CONTROL::x_prev[i] = x;

            Motion_control_set_PWM(i, (int)(x / kPwmM));
            MC_STU_RGB_set_latch(i, 0xA0u, 0x2Du, 0xFFu, time_now, 1u);
        }
        else if (manual_empty_pull)
        {
            int32_t x = MOTOR_CONTROL[i].dir * (700 * kPwmM);   /* 毫 PWM */
            if (x * MOTOR_CONTROL[i].dir < 0) x = 0;

            MOTOR_CONTROL[i].PID_speed.clear();
            MOTOR_CONTROL[i].PID_pressure.clear();
            MOTOR_CONTROL[i].pwm_zeroed = (x == 0) ? 1u : 0u;
            _MOTOR_CONTROL::x_prev[i] = x;

            Motion_control_set_PWM(i, (int)(x / kPwmM));
        }
        else if (have_time_step)
        {
            MOTOR_CONTROL[i].run(time_us, time_now);
        }

        uint8_t r = 0u, g = 0u, b = 0u;
        bool is_filament_rgb = false;

        const uint8_t pct = MC_PULL_pct[i];

        int hi_thr = MC_PULL_DEADBAND_PCT_HIGH;

        const filament_motion_enum m = MOTOR_CONTROL[i].motion;

        bool hi_hold =
            (m == filament_motion_enum::filament_motion_send) ||
            (m == filament_motion_enum::filament_motion_before_on_use) ||
            (m == filament_motion_enum::filament_motion_stop_on_use);

        if (!hi_hold && (m == filament_motion_enum::filament_motion_pressure_ctrl_on_use))
        {
            const uint64_t t0 = MOTOR_CONTROL[i].on_use_handoff_t0_ms;
            if (t0 != 0ull && (time_now - t0) < ON_USE_HANDOFF_MS) hi_hold = true;
        }

        if (hi_hold)
        {
            hi_thr = (int)MC_LOAD_S2_HOLD_TARGET_PCT + 3;
            if (hi_thr > 100) hi_thr = 100;
            if (hi_thr < 0) hi_thr = 0;
        }

        if (!(m == filament_motion_enum::filament_motion_before_on_use) && (int)pct >= hi_thr)
        {
            r = 0x10u;
        }
        else if (pct <= 30u)
        {
            b = 0x10u;
        }
        else
        {
            const uint8_t key = MC_ONLINE_key_stu[i];

#if BMCU_ONLINE_LED_FILAMENT_RGB
    #if BMCU_DM_TWO_MICROSWITCH
            const bool show_filament_rgb = (key == 1u) && dm_loaded[i] && !dm_fail_latch[i];
    #else
            const bool show_filament_rgb = (key != 0u);
    #endif
            if (show_filament_rgb)
            {
                r = Acol.filament[i].color_R;
                g = Acol.filament[i].color_G;
                b = Acol.filament[i].color_B;
                is_filament_rgb = true;
            }
            else
#endif
            {
                if (key == 0u && all_empty)
                {
                    if ((uint8_t)(pct - 49u) <= 2u) { r = 0x10u; g = 0x08u; }
                }
            }
        }

        // V11: 通道已插入但校准数据无效时以红色提示（需重新校准）
        if (filament_channel_inserted[i] && !MC_PULL_calibration_is_valid(i)) {
            r = 0x18u; g = b = 0u; is_filament_rgb = false;
        }

        // 亮度策略：所有通道灯固定 10% 亮度，任何状态不变化（夜间不刺眼）
        RGBOUT[i].set_brightness(26);
        MC_PULL_ONLINE_RGB_set(i, r, g, b, is_filament_rgb);
    }
}

bool Motion_control_filament_present(uint8_t ch)
{
    return ch < kChCount && filament_channel_inserted[ch] && MC_ONLINE_key_stu[ch] != 0u;
}

/**
 * @brief 运动控制主循环函数（每次主循环调用一次）
 *        读取传感器→更新AS5600→处理脱料→校准重置检测→执行运动→输出LED
 */
void Motion_control_run(int error)
{
    const uint64_t now_ticks64 = time_ticks64();
    const uint32_t now_ticks   = (uint32_t)now_ticks64;
    const uint64_t now_ms      = time_ms_fast_from_ticks64(now_ticks64);

    MC_PULL_ONLINE_read(now_ticks);

    const uint8_t loaded_ch = ams_state_get_loaded();
    if ((loaded_ch < kChCount) && (MC_ONLINE_key_stu[loaded_ch] == 0u))
        ams_state_set_unloaded(loaded_ch);

    auto &A = ams[motion_control_ams_num];

    for (uint8_t ch = 0; ch < kChCount; ch++)
    {
        const uint8_t ks = MC_ONLINE_key_stu[ch];
        if (ks == 0u)
        {
            if (!error)
            {
                if (A.now_filament_num == ch)
                {
                    if (A.filament[ch].motion == _filament_motion::send_out)
                        MOTOR_CONTROL[ch].set_motion(filament_motion_enum::filament_motion_stop, 100, now_ms);
                }
            }

            if (g_on_use_jam_latch[ch])
            {
                g_on_use_low_latch[ch] = 0u;
                g_on_use_jam_latch[ch] = 0u;
            }

            g_on_use_hi_pwm_us[ch] = 0u;
        }
    }

    if (!error)
    {
        const uint8_t n = A.now_filament_num;

        if ((n < kChCount) && filament_channel_inserted[n] && g_on_use_jam_latch[n])
        {
            const _filament_motion m = A.filament[n].motion;

            if (m == _filament_motion::on_use || m == _filament_motion::send_out)
                A.pressure = 0xF06Fu;
        }
    }

    if ((error <= 0) && all_no_filament())
    {
        int pressed = -1;

        for (uint8_t ch = 0; ch < kChCount; ch++)
        {
            if (!filament_channel_inserted[ch]) continue;

            const int     pct = (int)MC_PULL_pct[ch];
            const int32_t v   = MC_PULL_stu_raw[ch];   /* 微伏 µV */

            const bool hard_blue =
                (pct <= CAL_RESET_PCT_THRESH) ||
                (v <= (1650000 - CAL_RESET_V_DELTA_UV)) ||
                (v <= (MC_PULL_V_MIN[ch] + CAL_RESET_NEAR_MIN_UV));

            if (hard_blue) { pressed = (int)ch; break; }
        }

        uint32_t tpm = time_hw_tpms;
        if (!tpm) tpm = 1u;

        if (pressed >= 0)
        {
            if (g_hold_ch != pressed)
            {
                g_hold_ch = pressed;
                g_hold_t0_ticks = now_ticks;
            }
            else
            {
                if ((uint32_t)(now_ticks - g_hold_t0_ticks) >= (uint32_t)CAL_RESET_HOLD_MS * tpm)
                    calibration_reset_and_reboot();
            }
        }
        else
        {
            g_hold_ch = -1;
            g_hold_t0_ticks = 0u;
        }
    }
    else
    {
        g_hold_ch = -1;
        g_hold_t0_ticks = 0u;
    }

    AS5600_distance_updata(now_ticks);

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (MC_ONLINE_key_stu[i] != 0u) A.filament[i].online = true;
        else if ((filament_now_position[i] == filament_redetect) || (filament_now_position[i] == filament_pulling_back))
            A.filament[i].online = true;
        else
            A.filament[i].online = false;
    }

    motor_motion_run(error, now_ms, now_ticks);

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1))
            MC_STU_RGB_set(i, 0xFF, 0x00, 0x00);
    }
}

/* ==================== PWM 硬件初始化 ==================== */

/**
 * @brief 初始化PWM输出硬件
 *        配置TIM2/TIM3/TIM4为PWM模式，设置GPIO引脚复用功能
 *        PWM频率=72MHz/(999+1)/(1+1)=36kHz，占空比0~1000对应0~100%
 */
void MC_PWM_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                    GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Period        = 999;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/** @brief 计算两个AS5600角度值的差值（处理0/4096跨越） */
static inline int M5600_angle_dis(int16_t angle1, int16_t angle2)
{
    int d = (int)angle1 - (int)angle2;
    if (d >  2048) d -= 4096;
    if (d < -2048) d += 4096;
    return d;
}

/**
 * @brief 电机方向校准测试
 *        给每个未校准通道施加正向PWM，通过AS5600检测旋转方向
 *        确定dir=+1或-1并保存到Flash
 */
static void MOTOR_get_dir()
{
    int  dir[4]     = {0,0,0,0};
    bool test[4]    = {false,false,false,false};
    uint8_t phase[4] = {0,0,0,0};
    int8_t candidate[4] = {0,0,0,0};
    constexpr int detect_count = 163;
    bool any_detect = false;
    bool any_change = false;
    bool timed_out  = false;

    const bool have_data = Motion_control_read();
    if (!have_data)
    {
        for (uint8_t i = 0; i < kChCount; i++)
            Motion_control_data_save.Motion_control_dir[i] = 0;
    }

    int16_t last_angle[4] = {0,0,0,0};
    for (uint8_t i = 0; i < kChCount; i++)
    {
        dir[i] = Motion_control_data_save.Motion_control_dir[i];
    }

    // Start test tylko tam, gdzie:
    // - AS5600 online
    // - kanał fizycznie wpięty
    // - dir nieznany (0)
    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (AS5600_is_good(i) && filament_channel_inserted[i] &&
            MC_PULL_calibration_is_valid(i) && MC_ONLINE_key_stu[i] == 0u && (dir[i] == 0))
        {
            Motion_control_set_PWM(i, 0);
            test[i] = true;
        }
    }

    // jeśli nie ma nic do testowania -> nie rób NIC, nie zapisuj, nie psuj
    if (!(test[0] || test[1] || test[2] || test[3]))
        return;

    // czekaj max 2s na ruch (200 * 10ms)
    for (int t = 0; t < 200; t++)
    {
        delay(10);
        const uint8_t valid = MC_AS5600.updata_angle();

        bool done = true;

        for (uint8_t i = 0; i < kChCount; i++)
        {
            if (!test[i]) continue;

            // jeśli czujnik zniknął po drodze -> abort kanału (nie zapisuj)
            if (!(valid & (1u << i)) || MC_AS5600.magnet_stu[i] == AS5600_soft_IIC_many::offline)
            {
                Motion_control_set_PWM(i, 0);
                test[i] = false;
                continue;
            }

            const int16_t angle = (int16_t)MC_AS5600.raw_angle[i];
            if (phase[i] == 0u)
            {
                last_angle[i] = angle;
                phase[i] = 1u;
                done = false;
                continue;
            }
            const int angle_dis = M5600_angle_dis(angle, last_angle[i]);
            if (phase[i] == 1u)
            {
                last_angle[i] = angle;
                if (angle_dis <= detect_count / 4 && angle_dis >= -detect_count / 4)
                {
                    phase[i] = 2u;
                    Motion_control_set_PWM(i, 1000);
                }
                done = false;
                continue;
            }
            if (phase[i] >= 3u)
            {
                if ((candidate[i] > 0 && angle_dis > detect_count / 4) ||
                    (candidate[i] < 0 && angle_dis < -detect_count / 4))
                {
                    if (phase[i] == 3u)
                    {
                        phase[i] = 4u;
                        done = false;
                        continue;
                    }
                    dir[i] = candidate[i];
                    any_detect = true;
                }
                test[i] = false;
                continue;
            }

            if ((angle_dis > detect_count) || (angle_dis < -detect_count))
            {
                Motion_control_set_PWM(i, 0);

                // AS5600 odwrotnie względem magnesu
                candidate[i] = (angle_dis > 0) ? 1 : -1;
                phase[i] = 3u;
                done = false;
            }
            else
            {
                done = false;
            }
        }

        if (done) break;
        if (t == 199) timed_out = true;
    }

    // stop dla niedokończonych
    for (uint8_t i = 0; i < kChCount; i++)
        if (test[i]) Motion_control_set_PWM(i, 0);

    // zaktualizuj tylko tam, gdzie faktycznie zmieniło się dir
    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (dir[i] != Motion_control_data_save.Motion_control_dir[i])
        {
            Motion_control_data_save.Motion_control_dir[i] = dir[i];
            any_change = true;
        }
    }

    // zapis tylko jeśli była realna detekcja ruchu (dir => ±1)
    // Jak brak 24V i nic się nie ruszyło -> any_detect=false -> NIE zapisujemy.
    if (any_detect && any_change)
    {
        Motion_control_save();
    }
    else
    {
        (void)timed_out;
    }
}

/** @brief 电机初始化：初始化PWM→校准方向→设置默认参数 */
static void MOTOR_init()
{
    MC_PWM_init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    MOTOR_get_dir();

    /* 回抽目标步数：由回抽长度（µm）换算一次并缓存。
     * 锚点路径下该值只能在运行期取得（volatile 读取，编译期无法折叠），
     * 故统一放在这里初始化，保证任何使用点（状态机 / 回退目标）都已就绪。 */
    kPullBackCount = distance_counts(motion_control_pull_back_um);
    for (uint8_t i = 0; i < kChCount; i++)
    {
        filament_pull_back_target[i] = kPullBackCount;
    }

    for (uint8_t i = 0; i < kChCount; i++)
    {
        Motion_control_set_PWM(i, 0);
        MOTOR_CONTROL[i].set_pwm_zero(500 * kPwmM);
        MOTOR_CONTROL[i].dir = (int32_t)Motion_control_data_save.Motion_control_dir[i];
    }
}

/**
 * @brief 运动控制模块总初始化
 *        读取Flash校准→初始化ADC→初始化AS5600→初始化电机
 *        设置AMS在线状态、初始化DM自动加载状态、建立初始编码器基准
 */
void Motion_control_init()
{
    auto &A = ams[motion_control_ams_num];
    A.online   = true;
    A.ams_type = 0x03;

    (void)Motion_control_read();

    MC_PULL_ONLINE_init();
    MC_PULL_ONLINE_read(time_ticks32());

    #if BMCU_DM_TWO_MICROSWITCH
        for (uint8_t ch = 0; ch < kChCount; ch++)
        {
            if (!filament_channel_inserted[ch])
            {
                dm_loaded[ch]            = 1u;
                dm_fail_latch[ch]        = 0u;
                dm_auto_fields_reset(ch);
                dm_loaded_drop_t0_ms[ch] = 0ull;
                dm_autoload_gate[ch]     = 0u;
                continue;
            }

            const uint8_t ks = MC_ONLINE_key_stu[ch];

            dm_autoload_gate[ch] = 1u;
            dm_loaded[ch] = (ks == 1u) ? 1u : 0u;

            dm_fail_latch[ch]        = 0u;
            dm_auto_state[ch]        = DM_AUTO_IDLE;
            dm_auto_try[ch]          = 0u;
            dm_auto_t0_ms[ch]        = 0ull;
            dm_auto_remain_count[ch]     = 0u;
            dm_auto_last_count[ch]       = 0u;
            dm_loaded_drop_t0_ms[ch] = 0ull;
        }
    #endif

    MC_AS5600.init(AS5600_SCL_PORT, AS5600_SCL_PIN,
               AS5600_SDA_PORT, AS5600_SDA_PIN,
               4);
    const uint8_t angle_valid = MC_AS5600.updata_angle();
    MC_AS5600.updata_stu();

    for (uint8_t i = 0; i < kChCount; i++)
    {
        const bool ok = (angle_valid & (1u << i)) && MC_AS5600.online[i] && (MC_AS5600.magnet_stu[i] != AS5600_soft_IIC_many::offline);
        g_as5600_good[i]     = ok ? 1u : 0u;
        g_as5600_fail[i]     = ok ? 0u : kAS5600_FAIL_TRIP;
        g_as5600_okstreak[i] = ok ? kAS5600_OK_RECOVER : 0u;
    }

    for (uint8_t i = 0; i < kChCount; i++)
    {
        as5600_distance_save[i] = MC_AS5600.raw_angle[i];
        filament_now_position[i] = filament_idle;
    }

    MOTOR_init();
}

bool Motion_control_is_reset_safe(void)
{
    return true;
}

bool Motion_control_get_channel_telemetry(uint8_t channel, MotionControlChannelTelemetry* output)
{
    if (channel >= kChCount || output == nullptr) return false;
    output->raw_angle = MC_AS5600.raw_angle[channel];
    output->position_delta = 0;
    output->motor_pwm = 0;
    output->sensor_online = MC_AS5600.online[channel] ? 1u : 0u;
    output->motion_fault = 0;
    output->sensor_good = g_as5600_good[channel];
    output->controller_motion = (uint8_t)ams[motion_control_ams_num].filament[channel].motion;
    return true;
}

float Motion_control_get_filament_meters(uint8_t channel)
{
    if (channel >= kChCount) return 0.0f;
    /* 【定点化】原为 (float)filament_now_position[channel] 的隐式转换（会拉进 __floatsisf）。
     * 这里用纯位运算装配同一数值，保证整份固件一个软浮点助手都不引入。 */
    return fx_q_mag_to_float((uint32_t)filament_now_position[channel] << 24, 24u, 0u);
}
