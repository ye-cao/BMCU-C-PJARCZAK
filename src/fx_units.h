/**
 * @file fx_units.h
 * @brief 定点化单位约定与通用助手（彻底消除软浮点）
 *
 * 【为什么需要】
 *   CH32V203C8T6 是 RV32IMAC 内核，**没有硬件 FPU**。固件里任何 float 运算
 *   都会链接 libgcc 的软浮点助手：
 *       12 个单精度函数 4284 B (__addsf3 872 / __subsf3 908 / __mulsf3 732 / __divsf3 746
 *                              / __floatsisf 204 / __floatunsisf 188 / __lesf2 152 / __gesf2 150
 *                              / __fixsfsi 96 / __eqsf2 94 / __fixunssfsi 82 / __unordsf2 60)
 *       + __floatdisf 326 B + __floatundisf 302 B  (int64 → float)
 *       合计约 4.9 KB Flash，占 60 KB 总容量的 8%。
 *   由于链接器对库函数是**整函数链接**，只要代码里还剩 1 个调用点，整个助手就会被保留，
 *   所以必须"一处不剩"地全部定点化才能把这块空间收回来。
 *
 * 【单位约定】（全工程统一，改动代码时务必遵守）
 *   - 电压        int32_t  微伏 (µV)        1.65 V = 1650000
 *   - 拉力百分比  int32_t  厘百分比 (0.01%) 50 %   = 5000，范围 0 ~ 10000
 *   - PWM 输出    int32_t  毫 PWM (0.001)   1000(100 %) = 1000000，范围 -1000000 ~ 1000000
 *   - 电机速度    int32_t  微米/秒 (µm/s)   60 mm/s = 60000
 *   - 时间步长    int32_t  微秒 (µs)
 *   - 位移/距离   优先用 AS5600 计数 (count)；需要绝对长度时用微米 (µm)
 *
 * 【溢出铁律】
 *   所有定点乘法的中间结果都必须落在 int32 内（< 2^31）。
 *   若某处不得不引入 int64 中间结果，会连带拉进 __muldi3 / __divdi3（各数百字节），
 *   "省下来的还不够付回去"——务必改算法（先移位/先降精度）而不是改用 int64。
 *   同理，变量除数会引入 __divsi3/__udivsi3，能写成常量除法就不要写成变量除法。
 *
 * 【编译期换算技巧】
 *   参数常量若原来是非整数（如 50.2f），请用
 *       static constexpr int32_t kXxx = (int32_t)(50.2f * 100.0f);
 *   让编译器在编译期完成 float→int 的定值换算：与旧行为逐位一致，且**不产生任何运行期浮点代码**。
 */
#ifndef FX_UNITS_H
#define FX_UNITS_H

#include <stdint.h>

/* ==================== 通用整数助手 ==================== */

/** @brief 定点绝对值（避免依赖 fabsf 与可能的库版 abs） */
static inline int32_t fx_abs(int32_t x) { return x < 0 ? -x : x; }

/** @brief 定点限幅 */
static inline int32_t fx_clamp(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
 * @brief 把绝对值定点量按 IEEE-754 单精度位布局装配成 float（纯位运算，无助手调用）
 * @param q     绝对值定点量，要求 q < 2^31（即 0 <= q/2^24 < 128）
 * @param shift 定点小数位数（Q(32-shift) 格式），例如 Q24 传 24
 * @param sign  符号位（0x80000000 表示负数）
 * @return 对应的 float
 */
static inline float fx_q_mag_to_float(uint32_t q, uint32_t shift, uint32_t sign)
{
    if (q == 0u) return 0.0f;

    /* 取最高有效位序号 e，使 2^e <= q < 2^(e+1) */
    const uint32_t e = 31u - (uint32_t)__builtin_clz(q);

    /* 规格化尾数：把最高位对齐到 bit23；偏大时右移截断（相对误差 < 2^-23） */
    uint32_t mant = (e >= 23u) ? (q >> (e - 23u)) : (q << (23u - e));
    mant &= 0x7FFFFFu;

    /* 指数域 = (e - shift) + 127；q < 2^31 且 shift >= 24 ⇒ 指数域 <= 134，不会溢出 */
    union { uint32_t u; float f; } cvt;
    cvt.u = sign | (((e + 127u - shift) & 0xFFu) << 23) | mant;
    return cvt.f;
}

/* ==================== 必须向外输出 float 的场合 ==================== */

/** @brief AS5600 每米对应的计数（= 1e6 µm ÷ 5.752428 µm/count = 173839.64 → 173840）
 *  @note 必须与 Motion_control.cpp 的 kAS5600_CNT_PER_M 保持一致 */
#define FX_AS5600_CNT_PER_M   173840

/**
 * @brief AS5600 累计计数 → 耗材米数（float）
 *
 * 这是本工程**唯一**必须产出 float 的场合：`_filament::meters` 字段的 4 字节
 * IEEE-754 原样 memcpy 进打印机总线报文（`bambu_bus_ams.cpp` 的 package_send->meters），
 * 属协议要求，无法改为定点。
 *
 * 原实现为 `1.0f - (float)count * kAS5600_M_PER_CNT`，会拉进 __floatdisf / __mulsf3 /
 * __subsf3 三个助手（合计约 1.9 KB）。这里改为纯整数换算 + 位运算装配，
 * **一个助手都不引入** —— 这是本方案能否装下 OLED 变体的关键。
 *
 * @param counts 累计计数（int32；调用方负责先把 int64 里程表限幅到 int32 范围）
 * @return 耗材米数（可为负，对应计数超过初始位置的情形）
 */
static inline float fx_meters_from_counts(int32_t counts)
{
    /* counts > FX_AS5600_CNT_PER_M ⇒ 米数为负 */
    const uint32_t sign = (counts > FX_AS5600_CNT_PER_M) ? 0x80000000u : 0u;

    int32_t d = (int32_t)((counts > FX_AS5600_CNT_PER_M)
                          ? ((int64_t)counts - FX_AS5600_CNT_PER_M)
                          : ((int64_t)FX_AS5600_CNT_PER_M - counts));

    /* 限幅到约 12.6 米（= 2.2e6 计数），保证 d<<24 安全落在 uint64 内 */
    if (d > 2200000) d = 2200000;
    if (d < 0) d = 0;

    /* 米数 = d / FX_AS5600_CNT_PER_M，直接求 Q24 全商：
     *     q = (d << 24) / FX_AS5600_CNT_PER_M   （向下取整）
     * d ≤ 2.2e6 ⇒ d<<24 ≈ 3.7e13，必须用 uint64 中间量；该除法复用已链接的
     * __udivdi3（由 time_ms_fast_from_ticks64 引入），不额外付费。
     * 【为什么不用"整数部分 + 小数部分分别转换"】原先的
     *     frac = rem*9652/100 在 rem 接近 173840 时会 ≥ 2^24（该常数略大于真值
     *     2^24/173840×100 = 9651.868），使米数出现 ~1.1e-4 的台阶。
     * 全商写法只有一次 floor，误差 ≤ 1/2^24 相对量级。 */
    const uint32_t q = (uint32_t)(((uint64_t)(uint32_t)d << 24) / (uint64_t)FX_AS5600_CNT_PER_M);

    return fx_q_mag_to_float(q, 24u, sign);
}

/* ==================== 浮点比较的整数等价形式 ==================== */

/**
 * @brief 把 IEEE-754 单精度位模式映射为**单调递增**的无符号整数键
 *        （同号内保持序关系，且所有负数键 < 所有正数键）
 */
static inline uint32_t fx_float_sort_key(float x)
{
    union { float f; uint32_t u; } cvt;
    cvt.f = x;
    const uint32_t b = cvt.u;
    /* 正数：最高位置 1；负数：整体取反（负号 float 位越大数值越小） */
    return (b & 0x80000000u) ? ~b : (b | 0x80000000u);
}

/**
 * @brief `meters <= thresh` 的等价整数判断（不引入 __lesf2/__ltsf2）
 * @note  thresh 必须是编译期 float 字面量（如 0.05f），由编译器常量折叠，
 *        运行期不产生任何浮点指令。meters 须为有限值（本工程由
 *        fx_meters_from_counts 产出，必然有限）。
 */
static inline bool fx_meters_le(float meters, float thresh)
{
    return fx_float_sort_key(meters) <= fx_float_sort_key(thresh);
}

#endif /* FX_UNITS_H */
