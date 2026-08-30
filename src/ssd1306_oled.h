/**
 * @file ssd1306_oled.h
 * @brief SSD1306 OLED 驱动头文件（硬件 I2C2）
 *
 * SSD1306 0.96" 128x64 OLED 显示屏驱动。
 * 通过硬件 I2C2（PB10/SCL, PB11/SDA）与 AHT20/SHT45 共用总线通信。
 * 地址 0x3C（SA0 接地），与 AHT20(0x38)/SHT45(0x44) 不冲突。
 *
 * 显示规格（必须严格遵守）：
 *   分辨率 0.96" 128x64 => 4 行，每行 16 字符（行高 16px，列宽 8px）
 *   覆盖显示前必须先 clear_line / clear，否则字符叠加乱屏
 *   显示位置：列 = 字符序号 x 8，行 = 行号 x 16
 *
 * 默认编译：头文件顶部默认 #define BMCU_OLED
 *   运行时自动探测屏是否存在（ACK），无屏则跳过显示、零影响
 *   若不需要 OLED，注释掉头文件顶部的 #define BMCU_OLED 即可
 *
 * 分层设计：
 *   驱动层（driver）：init / clear / clear_line / show_char / show_text / 坐标换算
 *   内容层（content）：draw_aht20 / draw_channels / draw_message / notify_action / tick
 *
 * 字模：移植自江协科技 OLED 库 8x16 ASCII（可见字符 0x20~0x7E）
 */

#pragma once

#include <stdint.h>
#include "ams.h"           // _filament_motion / _filament_type / ams[]
#include "tpu_params.h"    // TPU_FIXED_ID[4] 写死型号

// BMCU_OLED 由 platformio.ini build_flags 或 build_fast.py 定义
// 不定义时整个 OLED 驱动编译为空（零开销）

#if BMCU_OLED

// ---------- 屏幕几何常量 ----------
#define OLED_W       128u
#define OLED_H       64u
#define OLED_LINE_H  16u   // 每行 16 像素（8x16 字模）
#define OLED_LINES   (OLED_H / OLED_LINE_H)   // 4 行
#define OLED_COLS    16u   // 每行 16 个 8px 字符

// OLED I2C 地址（7 位）。SA0 接地 = 0x3C（写地址 0x78）。可通过 -DBMCU_OLED_ADDR 覆盖
#ifndef BMCU_OLED_ADDR
#define BMCU_OLED_ADDR  0x3Cu
#endif

class SSD1306_OLED
{
public:
    // ===== 驱动层（纯屏幕操作，无业务逻辑）=====

    // 初始化 OLED：发送完整初始化命令序列并清屏。
    // 运行时探测 ACK，无屏则 s_ready=false，后续所有操作跳过。
    static void init();

    // 运行时探测 OLED ACK（不修改 s_ready，仅返回是否应答）。供热插拔/掉线检测
    static bool probe_ack();

    // 整屏清（全黑）
    static void clear();

    // 清第 row 行（0..3），避免整屏闪烁
    static void clear_line(uint8_t row);

    // 在 (row, col) 显示单个 8x16 字符（row∈[0,3], col∈[0,15]）
    static void show_char(uint8_t row, uint8_t col, char ch);

    // 在 (row, col) 显示字符串，自动截断到行宽（16 字符）
    static void show_text(uint8_t row, uint8_t col, const char* str);

    // 当前 OLED 是否初始化成功
    static bool is_ready() { return s_ready; }

    // 标记 OLED 不在线（供底层写函数调用）
    static void set_not_ready() { s_ready = false; }

    // ===== 内容层（界面画面）=====

    // 温湿度画面：行0=标题，行1=T/H，行2=通讯状态，行3=TPU型号摘要
    static void draw_aht20(bool aht20_present, bool online, float temperature_c, float humidity_percent, bool comm_ok);

    // 通用提示画面（4 行文本）
    static void draw_message(const char* line0, const char* line1,
                             const char* line2, const char* line3);

    // ===== 多页面轮询 + 动作覆盖显示 =====

    // 页面索引（轮询顺序）
    enum class oled_page : uint8_t
    {
        page_aht20 = 0,   // 温湿度（含 TPU 摘要）
        page_channels,    // 四通道概览
        page_count
    };

    // 动作类型（用于覆盖显示）
    enum class oled_action : uint8_t
    {
        action_none = 0,
        action_load,     // 进料
        action_unload,   // 退料
        action_feed,     // 送料中
        action_idle      // 空闲/停止
    };

    // 通道动作通知：由业务层调用，触发 OLED 立即覆盖显示
    static void notify_action(uint8_t ch, oled_action act);

    // 主循环每秒调用：处理页面轮询与动作覆盖的调度与绘制
    static void tick(bool aht20_present, bool aht20_online,
                     float temperature_c, float humidity_percent,
                     bool comm_ok);

    // 四通道概览页
    static void draw_channels();

    // RGB 转 3 字母颜色简写
    static const char* color_name(uint8_t r, uint8_t g, uint8_t b);

private:
    static inline bool s_ready = false;

    // 行缓存（去闪烁）：仅当内容变化才重写该行
    static inline char s_line_buf[OLED_LINES][OLED_COLS + 1u];

    // 页面轮询状态
    static inline oled_page s_page = oled_page::page_aht20;
    static inline uint64_t  s_page_next_ms = 0;
    static constexpr  uint64_t OLED_PAGE_DWELL_MS = 5000u;
    static constexpr  uint64_t OLED_ACTION_HOLD_MS = 2500u;

    // 动作覆盖状态
    static inline oled_action s_action = oled_action::action_none;
    static inline uint8_t     s_action_ch = 0xFFu;
    static inline uint64_t    s_action_until_ms = 0;

    static const char* motion_label(_filament_motion m);
    static const char* material_label(uint8_t ch);
    static void draw_line_if_changed(uint8_t row, const char* text);
    static void write_cmd(uint8_t c);
    static void write_data(uint8_t d);
    static void set_pos(uint8_t page, uint8_t col);
};

#endif // BMCU_OLED
