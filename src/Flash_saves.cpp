/**
 * @file Flash_saves.cpp
 * @brief Flash 非易失性存储(NVM)管理实现
 *
 * 实现了 CH32V203C8T6 MCU 最后 4KB Flash 扇区的读写管理,
 * 包含耗材信息的追加日志存储、状态的环形日志存储、
 * 校准和运动参数的标准 NVM256 页存储。
 *
 * 关键设计:
 * - 耗材信息: 每槽位 256 字节页, 内含 6 个 40 字节槽位 (slot)
 *   采用追加写入, 页满时擦除重写
 * - 状态记录: 10 页环形日志, 每槽位 8 字节 (2个 uint32),
 *   通过序列号找到最新记录
 * - 校准/运动: 标准 NVM256 页格式 (头部+载荷+CRC32)
 */

#include "Flash_saves.h"
#include "hal/irq_wch.h"
#include "hal/time_hw.h"
#include <string.h>
#include "_bus_hardware.h"

#include "ch32v20x_rcc.h"
#include "ch32v20x_crc.h"
#include "ch32v20x_flash.h"

/** 编译期检查: Flash_FilamentInfo 必须恰好 32 字节, 确保槽位布局正确 */
static_assert(sizeof(Flash_FilamentInfo) == 32u, "Flash_FilamentInfo must be 32 bytes");
/** 编译期检查: Flash_FilamentInfo 必须 4 字节对齐, 确保 Flash 编程安全 */
static_assert(alignof(Flash_FilamentInfo) >= 4u, "Flash_FilamentInfo must be 4-byte aligned");

/**
 * @brief 使用 CH32V203 硬件 CRC32 引擎计算校验值
 *
 * 以 32 位字为单位输入数据, 利用片上 CRC 外设加速计算。
 * 注意: 数据长度必须为 4 的倍数。
 *
 * @param data 输入数据指针 (必须 4 字节对齐)
 * @param bytes 数据字节数 (必须为 4 的倍数)
 * @return uint32_t CRC32 校验值
 */
static uint32_t crc32_hw_words(const void* data, uint32_t bytes)
{
    const uint32_t* p = (const uint32_t*)data;
    CRC->CTLR = 1u;  // 复位 CRC 寄存器, 开始新计算
    for (uint32_t i = 0u; i < (bytes >> 2); i++) CRC->DATAR = p[i]; // 逐字写入数据
    return CRC->DATAR; // 读取最终 CRC 结果
}

/**
 * @brief 根据耗材索引计算其在 Flash 中的页基地址
 * @param filament_idx 耗材索引 (0-3)
 * @return uint32_t 该耗材对应的 256 字节页起始地址
 */
static inline uint32_t ams_fil_page(uint8_t filament_idx)
{
    return FLASH_NVM_AMS_ADDR + (uint32_t)filament_idx * FLASH_NVM256_PAGE_SIZE;
}

/** @brief Flash 擦除后的字节模式: 0xE3 (CH32V203 特定的擦除后值) 重复填充 */
static constexpr uint32_t FLASH_ERASED_WORD = 0xE339E339u;

class Flash_clock_guard
{
    const uint32_t was_slow;
public:
    Flash_clock_guard() : was_slow(time_hw_slow) {
        if (!was_slow) time_hw_flash_clock(1u);
    }
    ~Flash_clock_guard() {
        if (!was_slow && !(FLASH->STATR & FLASH_FLAG_BSY)) time_hw_flash_clock(0u);
    }
};

/**
 * @brief 检查一个 32 位字是否为 Flash 擦除后的空白状态
 *
 * CH32V203 Flash 擦除后字节为 0xE3, 即 32 位字为 0xE339E339;
 * 同时也兼容标准的 0xFFFFFFFF 擦除模式。
 *
 * @param v 要检查的值
 * @return true 值为擦除状态 (空白)
 */
static inline bool flash_word_is_blank(uint32_t v)
{
    return v == FLASH_ERASED_WORD || v == 0xFFFFFFFFu;
}

/**
 * @brief 检查一段 Flash 区域是否已完全擦除
 * @param base_addr 起始地址 (必须 4 字节对齐)
 * @param bytes 检查字节数 (必须为 4 的倍数)
 * @return true 整段区域均为擦除状态
 */
static bool flash_range_is_erased(uint32_t base_addr, uint32_t bytes)
{
    Flash_clock_guard clock;
    const uint32_t* p = (const uint32_t*)base_addr;

    for (uint32_t i = 0u; i < (bytes >> 2); i++)
    {
        if (!flash_word_is_blank(p[i])) return false;
    }

    return true;
}

static constexpr uint32_t FLASH_ERASE_TIMEOUT_US = 32000u;
static constexpr uint32_t FLASH_PROGRAM_TIMEOUT_US = 4000u;
static bool g_flash_fault = false;

static bool flash_wait_ready(uint32_t timeout_us)
{
    const uint32_t started = time_ticks32();
    const uint32_t timeout = us_to_ticks32(timeout_us);
    while (FLASH->STATR & FLASH_FLAG_BSY)
        if ((uint32_t)(time_ticks32() - started) >= timeout) return false;
    return (FLASH->STATR & FLASH_FLAG_WRPRTERR) == 0u;
}

/**
 * @brief 擦除一页 256 字节 Flash (不写入)
 * @param page_addr 页起始地址 (必须 256 字节对齐)
 * @return true 擦除成功且验证通过
 */
static bool flash256_erase(uint32_t page_addr)
{
    if (page_addr & (FLASH_NVM256_PAGE_SIZE - 1u)) return false;
    if (g_flash_fault || !Flash_background_idle()) return false;
    Flash_clock_guard clock;
    const uint32_t irq = irq_save_wch();
    FLASH_Unlock_Fast();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    FLASH->CTLR |= 0x00020000u;
    FLASH->ADDR = page_addr;
    FLASH->CTLR |= 0x00000040u;
    const bool ok = flash_wait_ready(FLASH_ERASE_TIMEOUT_US);
    if (!(FLASH->STATR & FLASH_FLAG_BSY)) {
        FLASH->CTLR &= ~0x00020000u;
        FLASH_Lock_Fast();
        FLASH_Lock();
    }
    irq_restore_wch(irq);
    const bool verified = ok && flash_range_is_erased(page_addr, FLASH_NVM256_PAGE_SIZE);
    if (!verified) g_flash_fault = true;
    return verified;
}

/**
 * @brief 使用标准编程接口写入单个 32 位字
 *
 * 与快速编程不同, 此函数适用于少量字的写入。
 * 写入前检查: 地址对齐、数据是否已相同。
 *
 * @param addr 目标地址 (必须 4 字节对齐)
 * @param data 待写入的值
 * @return true 编程成功且验证通过
 */
static bool flash_word_prog_std(uint32_t addr, uint32_t data)
{
    if ((addr & 3u) || g_flash_fault || !Flash_background_idle()) return false;
    Flash_clock_guard clock;
    if (*(const volatile uint32_t*)addr == data) return true;
    const uint32_t irq = irq_save_wch();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    FLASH->CTLR |= 1u;
    *(volatile uint16_t*)addr = (uint16_t)data;
    bool ok = flash_wait_ready(FLASH_PROGRAM_TIMEOUT_US);
    if (ok) {
        *(volatile uint16_t*)(addr + 2u) = (uint16_t)(data >> 16);
        ok = flash_wait_ready(FLASH_PROGRAM_TIMEOUT_US);
    }
    if (!(FLASH->STATR & FLASH_FLAG_BSY)) {
        FLASH->CTLR &= ~1u;
        FLASH_Lock();
    }
    irq_restore_wch(irq);
    const bool verified = ok && *(const volatile uint32_t*)addr == data;
    if (!verified) g_flash_fault = true;
    return verified;
}

/**
 * @brief 从基地址开始连续编程多个 32 位字
 * @param base_addr 起始地址 (必须 4 字节对齐)
 * @param words 数据数组
 * @param count 字数
 * @return true 全部编程成功
 */
static bool flash_prog_words(uint32_t base_addr, const uint32_t* words, uint32_t count)
{
    if (base_addr & 3u) return false;

    for (uint32_t i = 0u; i < count; i++)
    {
        if (!flash_word_prog_std(base_addr + (i << 2), words[i]))
            return false;
    }

    return true;
}

/**
 * @brief 通用 NVM 256 字节页写入函数
 *
 * 组装完整的 256 字节页: [NVM256_HDR 头部] + [载荷数据] + [填充0xFF] + [CRC32]
 * CRC 覆盖前 252 字节 (头部 + 载荷 + 填充)。
 *
 * @param page_addr 页起始地址
 * @param magic 魔数标识
 * @param ver 版本号
 * @param rsv 保留字段值
 * @param payload 载荷数据指针 (可为 NULL)
 * @param len 载荷字节数
 * @return true 写入成功
 */
static bool nvm256_write(uint32_t page_addr, uint32_t magic, uint16_t ver, uint32_t rsv,
                         const void* payload, uint16_t len)
{
    // 载荷不能超过头部到 CRC 之间的空间
    if (len > (uint16_t)(NVM256_CRC_OFF - sizeof(NVM256_HDR))) return false;
    if (g_flash_fault || !Flash_background_idle()) return false;
    Flash_clock_guard clock;

    alignas(4) uint32_t w[64];       // 256 字节缓冲区 (64 个 uint32)
    uint8_t* b = (uint8_t*)w;
    memset(b, 0xFF, FLASH_NVM256_PAGE_SIZE); // 初始化为全 0xFF (Flash 空白值)

    // 填充头部
    NVM256_HDR h{};
    h.magic = magic;
    h.ver = ver;
    h.len = len;
    h.rsv = rsv;

    memcpy(b, &h, sizeof(h));       // 拷贝头部
    if (len) memcpy(b + sizeof(h), payload, len); // 拷贝载荷

    // 计算前 252 字节的 CRC32 并写入末尾
    const uint32_t crc = crc32_hw_words(b, NVM256_CRC_OFF);
    memcpy(b + NVM256_CRC_OFF, &crc, 4u);

    if (!flash256_erase(page_addr)) return false;
    if (!flash_prog_words(page_addr + 4u, w + 1u, 63u)) return false;
    return flash_word_prog_std(page_addr, w[0]);
}

/**
 * @brief 通用 NVM 256 字节页读取函数
 *
 * 读取并验证: 先检查 CRC32, 再验证魔数和版本号,
 * 最后拷贝载荷数据。
 *
 * @param page_addr 页起始地址
 * @param magic 期望的魔数
 * @param ver 期望的版本号
 * @param out 输出缓冲区
 * @param max_len 输出缓冲区最大长度
 * @param got_len 输出: 实际读取的载荷长度 (可为 NULL)
 * @param rsv_out 输出: 保留字段值 (可为 NULL)
 * @return true 读取成功且 CRC/魔数/版本均匹配
 */
static bool nvm256_read(uint32_t page_addr, uint32_t magic, uint16_t ver,
                        void* out, uint16_t max_len, uint16_t* got_len, uint32_t* rsv_out)
{
    if (FLASH->STATR & FLASH_FLAG_BSY) return false;
    Flash_clock_guard clock;
    const uint8_t* b = (const uint8_t*)page_addr;

    // 读取存储的 CRC32 值
    const uint32_t stored = *(const uint32_t*)(b + NVM256_CRC_OFF);
    if (flash_word_is_blank(stored)) return false; // 全空白表示无数据

    // 验证 CRC32 完整性
    const uint32_t crc = crc32_hw_words(b, NVM256_CRC_OFF);
    if (crc != stored) return false; // CRC 不匹配, 数据损坏

    // 读取并验证头部
    NVM256_HDR h{};
    memcpy(&h, b, sizeof(h));

    if (h.magic != magic) return false;  // 魔数不匹配
    if (h.ver != ver) return false;      // 版本不匹配
    if (h.len > max_len) return false;   // 载荷超出缓冲区

    // 拷贝载荷数据
    if (h.len) memcpy(out, b + sizeof(h), h.len);
    if (got_len) *got_len = h.len;
    if (rsv_out) *rsv_out = h.rsv;
    return true;
}

/** @brief 耗材槽位大小: 10 个 uint32 = 40 字节 (含魔数、32字节数据、CRC32) */
static constexpr uint32_t FIL_SLOT_WORDS = 10u;
/** @brief 耗材槽位字节数: 40 字节 */
static constexpr uint32_t FIL_SLOT_BYTES = FIL_SLOT_WORDS * 4u;
/** @brief 每页耗材槽数: 256 / 40 = 6 (余 16 字节未使用) */
static constexpr uint32_t FIL_SLOTS_PER_PAGE = 6u;
static constexpr uint32_t FIL_SCRATCH_ADDR = FLASH_NVM_BASE_ADDR + 14u * FLASH_NVM256_PAGE_SIZE;
static constexpr uint32_t FIL_SCRATCH_MAGIC = 0x32524340u;

/** 编译期检查: 6 个槽位不能超出 256 字节页边界 */
static_assert(FIL_SLOT_BYTES * FIL_SLOTS_PER_PAGE <= FLASH_NVM256_PAGE_SIZE, "FIL journal too large");

/**
 * @brief 获取耗材槽位在 Flash 中的页基地址
 * @param filament_idx 耗材索引 (0-3)
 * @return 该耗材对应的页起始地址
 */
static inline uint32_t fil_page_addr(uint8_t filament_idx)
{
    return ams_fil_page(filament_idx);
}

/**
 * @brief 将耗材信息打包到槽位字数组中
 *
 * 布局: w[0]=MAGIC_FIL, w[1..8]=Flash_FilamentInfo (32字节), w[9]=CRC32
 *
 * @param w[10] 输出: 打包后的 10 字数组
 * @param info 输入: 耗材信息
 */
static inline void fil_slot_pack(uint32_t w[FIL_SLOT_WORDS], const Flash_FilamentInfo* info)
{
    w[0] = MAGIC_FIL;                                    // 槽位魔数
    memcpy(&w[1], info, sizeof(*info));                  // 拷贝 32 字节耗材数据
    w[FIL_SLOT_WORDS - 1u] = crc32_hw_words(w, (FIL_SLOT_WORDS - 1u) * 4u); // CRC32 覆盖前 9 字
}

/**
 * @brief 验证耗材槽位数据的合法性
 *
 * 检查魔数和 CRC32, 若通过则拷贝出耗材信息。
 *
 * @param p 槽位数据指针 (10 个 uint32)
 * @param out 输出: 耗材信息 (可为 NULL, 仅做验证)
 * @return true 槽位数据有效
 */
static inline bool fil_slot_valid(const uint32_t* p, Flash_FilamentInfo* out)
{
    if (p[0] != MAGIC_FIL) return false;  // 魔数校验
    if (crc32_hw_words(p, (FIL_SLOT_WORDS - 1u) * 4u) != p[FIL_SLOT_WORDS - 1u]) return false; // CRC 校验
    if (out) memcpy(out, &p[1], sizeof(*out)); // 提取耗材信息
    return true;
}

/**
 * @brief 扫描一页耗材槽位, 找到最后有效数据和第一个空槽位
 *
 * 从头到尾扫描 6 个槽位, 记录最后一个有效槽位的耗材信息,
 * 以及遇到的第一个空白槽位位置 (用于下次追加写入)。
 *
 * @param base 页基地址
 * @param last 输出: 最后一个有效槽位的耗材信息
 * @param first_empty 输出: 第一个空槽位索引 (0~5), 全满则为 FIL_SLOTS_PER_PAGE
 * @return true 找到至少一个有效槽位
 */
static bool fil_scan_page(uint32_t base, Flash_FilamentInfo* last, uint32_t* first_empty)
{
    Flash_clock_guard clock;
    bool found = false;

    if (first_empty) *first_empty = FIL_SLOTS_PER_PAGE; // 默认: 全满

    for (uint32_t s = 0u; s < FIL_SLOTS_PER_PAGE; s++)
    {
        const uint32_t* p = (const uint32_t*)(base + s * FIL_SLOT_BYTES);

        bool blank = true;
        for (uint32_t i = 0u; i < FIL_SLOT_WORDS; i++)
            if (!flash_word_is_blank(p[i])) { blank = false; break; }
        if (blank)
        {
            if (first_empty && *first_empty == FIL_SLOTS_PER_PAGE)
                *first_empty = s;
            continue;
        }

        if (first_empty) *first_empty = FIL_SLOTS_PER_PAGE;
        Flash_FilamentInfo tmp;
        if (fil_slot_valid(p, &tmp))
        {
            if (last) *last = tmp;
            found = true;
        }
    }

    return found;
}

/** @brief 各耗材槽位是否有有效数据 (0/1) */
static uint8_t g_fil_have[4] = {0u, 0u, 0u, 0u};
/** @brief 各耗材槽位下一个可写入的槽位索引 */
static uint8_t g_fil_first_empty[4] = {0u, 0u, 0u, 0u};
/** @brief 各耗材槽位缓存的最新耗材信息 */
static Flash_FilamentInfo g_fil_last[4];

/**
 * @brief 初始化加载单个耗材槽位的缓存
 *
 * 扫描对应 Flash 页, 将最新耗材信息和空槽位位置
 * 加载到全局缓存变量中, 避免每次都读 Flash。
 *
 * @param filament_idx 耗材索引 (0-3)
 */
static void fil_cache_load_one(uint8_t filament_idx)
{
    Flash_FilamentInfo last;
    uint32_t first_empty = FIL_SLOTS_PER_PAGE;
    const uint32_t base = fil_page_addr(filament_idx);

    if (fil_scan_page(base, &last, &first_empty))
    {
        g_fil_have[filament_idx] = 1u;
        g_fil_first_empty[filament_idx] = (uint8_t)first_empty;
        memcpy(&g_fil_last[filament_idx], &last, sizeof(last));
        return;
    }

    // 无有效数据: 重置缓存
    g_fil_have[filament_idx] = 0u;
    g_fil_first_empty[filament_idx] = (uint8_t)first_empty;
    memset(&g_fil_last[filament_idx], 0, sizeof(g_fil_last[filament_idx]));
}

static constexpr uint32_t STA_TAG = 0xA6u;
static constexpr uint32_t STA_PAGE_FIRST = 6u;
static constexpr uint32_t STA_PAGE_COUNT = 8u;
static constexpr uint32_t STA_SLOT_BYTES = 8u;
/** @brief 每页状态记录条数: 256 / 8 = 32 条 */
static constexpr uint32_t STA_SLOTS_PER_PAGE = (FLASH_NVM256_PAGE_SIZE / STA_SLOT_BYTES);
/** @brief 状态记录总槽位数: 10 * 32 = 320 条 */
static constexpr uint32_t STA_TOTAL_SLOTS = (STA_PAGE_COUNT * STA_SLOTS_PER_PAGE);
static constexpr uint32_t CAL_BACKUP_ADDR = FLASH_NVM_BASE_ADDR + 15u * FLASH_NVM256_PAGE_SIZE;

/** @brief 当前序列号 (每次写入递增, 用于找到最新记录) */
static uint16_t g_sta_seq = 0u;
/** @brief 下一个可写入的状态槽位索引 */
static uint16_t g_sta_slot = 0u;
static uint16_t g_sta_saved_slot = 0u;
static uint8_t g_sta_have_saved = 0u;
/** @brief 上次保存的通道号 (用于去重), 0xFF 表示未保存 */
static uint8_t g_sta_saved_loaded = 0xFFu;
static bool g_sta_scanned = false;

/**
 * @brief 重置全部运行时缓存到初始状态
 *
 * 在全擦除或初始化时调用, 清空所有内存中的缓存变量。
 */
static void flash_runtime_cache_clear(void)
{
    memset(g_fil_have, 0, sizeof(g_fil_have));
    memset(g_fil_first_empty, 0, sizeof(g_fil_first_empty));
    memset(g_fil_last, 0, sizeof(g_fil_last));

    g_sta_seq = 0u;
    g_sta_slot = 0u;
    g_sta_saved_slot = 0u;
    g_sta_have_saved = 0u;
    g_sta_saved_loaded = 0xFFu;
    g_sta_scanned = false;
}

/**
 * @brief 完全擦除 NVM 4KB Flash 扇区
 *
 * 擦除后验证整段区域是否为空白, 然后重置所有缓存。
 * @return true 擦除成功
 */
bool Flash_NVM_full_clear(void)
{
    if (!Flash_background_idle()) return false;
    Flash_clock_guard clock;
    for (uint32_t i = 0u; i < FLASH_NVM_PAGE_COUNT; i++)
        if (!flash256_erase(FLASH_NVM_BASE_ADDR + i * FLASH_NVM256_PAGE_SIZE)) return false;

    flash_runtime_cache_clear();
    return true;
}

/**
 * @brief 根据页索引计算状态记录页的地址
 * @param page_i 页内相对索引 (0 ~ STA_PAGE_COUNT-1)
 * @return 该页的绝对 Flash 地址
 */
static inline uint32_t sta_page_addr(uint32_t page_i)
{
    return FLASH_NVM_BASE_ADDR + (STA_PAGE_FIRST + page_i) * FLASH_NVM256_PAGE_SIZE;
}

/**
 * @brief 根据全局槽位索引计算状态记录的绝对地址
 *
 * 将线性槽位号映射到页号 + 页内偏移。
 *
 * @param slot 全局槽位索引 (0 ~ STA_TOTAL_SLOTS-1)
 * @return 该槽位的绝对 Flash 地址
 */
static inline uint32_t sta_slot_addr(uint32_t slot)
{
    const uint32_t page_i = slot / STA_SLOTS_PER_PAGE;
    const uint32_t slot_i = slot - page_i * STA_SLOTS_PER_PAGE;
    return sta_page_addr(page_i) + slot_i * STA_SLOT_BYTES;
}

struct Flash_job
{
    uint32_t words[FIL_SLOT_WORDS];
    uint32_t addr;
    uint32_t started;
    uint16_t slot;
    uint8_t kind;
    uint8_t channel;
    uint8_t halfwords;
    uint8_t index;
    uint8_t phase;
    bool erase;
    bool rollover;
};

static Flash_job g_flash_job = {};

static uint8_t flash_job_halfword(const Flash_job& job)
{
    const uint8_t index = job.index + 2u;
    return index < job.halfwords ? index : index - job.halfwords;
}

bool Flash_background_idle(void)
{
    return !g_flash_job.kind && !(FLASH->STATR & FLASH_FLAG_BSY);
}

void Flash_background_run(void)
{
    Flash_job &job = g_flash_job;
    if (!job.kind) {
        if (time_hw_slow && !(FLASH->STATR & FLASH_FLAG_BSY)) {
            FLASH->CTLR &= ~(0x00020000u | 1u);
            FLASH_Lock_Fast();
            FLASH_Lock();
            time_hw_flash_clock(0u);
        }
        return;
    }
    if (job.phase) {
        if (FLASH->STATR & FLASH_FLAG_BSY) {
            const uint32_t timeout = job.phase == 1u ? FLASH_ERASE_TIMEOUT_US : FLASH_PROGRAM_TIMEOUT_US;
            if ((uint32_t)(time_ticks32() - job.started) >= us_to_ticks32(timeout)) g_flash_fault = true;
            return;
        }
        FLASH->CTLR &= ~(0x00020000u | 1u);
        FLASH_Lock_Fast();
        FLASH_Lock();
        bool ok = !g_flash_fault && !(FLASH->STATR & FLASH_FLAG_WRPRTERR);
        if (job.phase == 1u) {
            ok = ok && flash_range_is_erased(job.addr & ~(FLASH_NVM256_PAGE_SIZE - 1u), FLASH_NVM256_PAGE_SIZE);
            job.erase = false;
        } else {
            const uint8_t index = flash_job_halfword(job);
            const uint16_t expected = (uint16_t)(job.words[index >> 1] >> ((index & 1u) * 16u));
            ok = ok && *(const volatile uint16_t*)(job.addr + (uint32_t)index * 2u) == expected;
            ++job.index;
        }
        job.phase = 0u;
        time_hw_flash_clock(0u);
        if (!ok) {
            g_flash_fault = true;
            job.kind = 0u;
            return;
        }
        if (job.index == job.halfwords) {
            if (job.kind == 3u) {
                job.kind = 1u;
                job.addr = fil_page_addr(job.channel);
                job.words[0] = MAGIC_FIL;
                job.words[FIL_SLOT_WORDS - 1u] = crc32_hw_words(job.words, (FIL_SLOT_WORDS - 1u) * 4u);
                job.index = 0u;
                job.erase = true;
                return;
            }
            if (job.kind == 1u && job.rollover) {
                job.kind = 4u;
                job.addr = FIL_SCRATCH_ADDR;
                job.index = job.halfwords = 0u;
                job.erase = true;
                return;
            }
            if (job.kind == 1u || job.kind == 4u) {
                memcpy(&g_fil_last[job.channel], &job.words[1], sizeof(Flash_FilamentInfo));
                g_fil_have[job.channel] = 1u;
            } else {
                g_sta_seq = (uint16_t)(g_sta_seq + 1u);
                g_sta_saved_slot = job.slot;
                g_sta_saved_loaded = job.channel;
                g_sta_have_saved = 1u;
            }
            job.kind = 0u;
            return;
        }
    }
    if (g_flash_fault || !bus_background_ready()) return;
    const uint32_t irq = irq_save_wch();
    time_hw_flash_clock(1u);
    job.started = time_ticks32();
    if (job.erase) {
        FLASH_Unlock_Fast();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
        FLASH->CTLR |= 0x00020000u;
        FLASH->ADDR = job.addr & ~(FLASH_NVM256_PAGE_SIZE - 1u);
        job.phase = 1u;
        FLASH->CTLR |= 0x00000040u;
    } else {
        const uint8_t index = flash_job_halfword(job);
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
        FLASH->CTLR |= 1u;
        job.phase = 2u;
        *(volatile uint16_t*)(job.addr + (uint32_t)index * 2u) =
            (uint16_t)(job.words[index >> 1] >> ((index & 1u) * 16u));
    }
    irq_restore_wch(irq);
}

static bool fil_scratch_recover(void)
{
    if (g_flash_fault || !Flash_background_idle()) return false;
    Flash_clock_guard clock;
    const uint32_t* saved = (const uint32_t*)FIL_SCRATCH_ADDR;
    if ((saved[0] & ~3u) != FIL_SCRATCH_MAGIC ||
        crc32_hw_words(saved, (FIL_SLOT_WORDS - 1u) * 4u) != saved[FIL_SLOT_WORDS - 1u])
        return true;
    const uint8_t channel = saved[0] & 3u;
    Flash_FilamentInfo info;
    memcpy(&info, &saved[1], sizeof(info));
    const bool restored = g_fil_have[channel] && memcmp(&g_fil_last[channel], &info, sizeof(info)) == 0;
    g_fil_last[channel] = info;
    g_fil_have[channel] = 1u;
    if (!restored) {
        uint32_t words[FIL_SLOT_WORDS];
        fil_slot_pack(words, &info);
        const uint32_t addr = fil_page_addr(channel);
        if (!flash256_erase(addr) || !flash_prog_words(addr + 4u, words + 1u, FIL_SLOT_WORDS - 1u) ||
            !flash_word_prog_std(addr, words[0])) return false;
        g_fil_first_empty[channel] = 1u;
    }
    return flash256_erase(FIL_SCRATCH_ADDR);
}

void Flash_saves_init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE); // 使能硬件 CRC 时钟

    flash_runtime_cache_clear();
    g_flash_job = {};
    g_flash_fault = false;
    Flash_clock_guard clock;
    FLASH_Unlock();
    FLASH_Unlock_Fast();
    FLASH_Access_Clock_Cfg(FLASH_Access_SYSTEM_HALF);
    FLASH_Lock_Fast();
    FLASH_Lock();

    for (uint8_t i = 0u; i < 4u; i++)
        fil_cache_load_one(i);
    uint8_t loaded;
    if (!Flash_AMS_state_read(&loaded)) g_flash_fault = true;
    if (!fil_scratch_recover()) g_flash_fault = true;
}

/**
 * @brief 写入耗材信息到 Flash 指定槽位
 *
 * 流程:
 * 1. 参数校验
 * 2. 与缓存比较, 数据相同则跳过 (减少 Flash 磨损)
 * 3. 检查页内是否有空槽位, 无则擦除整页
 * 4. 打包数据并编程到下一个空槽位
 * 5. 更新运行时缓存
 *
 * @param filament_idx 耗材索引 (0-3)
 * @param info 输入: 要写入的耗材信息
 * @return true 写入成功
 */
bool Flash_AMS_filament_write(uint8_t filament_idx, const Flash_FilamentInfo* info)
{
    if (!info || filament_idx >= 4u) return false;
    if (g_flash_fault || !Flash_background_idle()) return false;

    // 去重: 与缓存中最新数据比较, 相同则跳过
    if (g_fil_have[filament_idx] &&
        memcmp(&g_fil_last[filament_idx], info, sizeof(*info)) == 0)
        return true;

    uint32_t slot = g_fil_first_empty[filament_idx];
    const bool erase = slot >= FIL_SLOTS_PER_PAGE;
    if (erase) slot = 0u;
    Flash_job &job = g_flash_job;
    job = {};
    job.kind = erase ? 3u : 1u;
    job.channel = filament_idx;
    job.addr = erase ? FIL_SCRATCH_ADDR : fil_page_addr(filament_idx) + slot * FIL_SLOT_BYTES;
    job.erase = erase;
    job.rollover = erase;
    job.halfwords = FIL_SLOT_WORDS * 2u;
    fil_slot_pack(job.words, info);
    if (erase) {
        job.words[0] = FIL_SCRATCH_MAGIC | filament_idx;
        job.words[FIL_SLOT_WORDS - 1u] = crc32_hw_words(job.words, (FIL_SLOT_WORDS - 1u) * 4u);
    }
    g_fil_first_empty[filament_idx] = (uint8_t)(slot + 1u);
    return false;
}

/**
 * @brief 从缓存读取指定耗材槽位的信息
 *
 * 不直接读 Flash, 而是从内存缓存中获取 (初始化时已加载)。
 *
 * @param filament_idx 耗材索引 (0-3)
 * @param out 输出: 耗材信息
 * @return true 读取成功, false 索引无效或无数据
 */
bool Flash_AMS_filament_read(uint8_t filament_idx, Flash_FilamentInfo* out)
{
    if (!out || filament_idx >= 4u) return false;
    if (!g_fil_have[filament_idx]) return false;

    memcpy(out, &g_fil_last[filament_idx], sizeof(*out));
    return true;
}

/**
 * @brief 擦除指定耗材槽位的全部 Flash 数据
 * @param filament_idx 耗材索引 (0-3)
 * @return true 擦除成功
 */
bool Flash_AMS_filament_clear(uint8_t filament_idx)
{
    if (filament_idx >= 4u) return false;
    if (!flash256_erase(fil_page_addr(filament_idx))) return false;

    // 清空缓存
    g_fil_have[filament_idx] = 0u;
    g_fil_first_empty[filament_idx] = 0u;
    memset(&g_fil_last[filament_idx], 0, sizeof(g_fil_last[filament_idx]));
    return true;
}

static uint32_t sta_find_slot(bool* erase)
{
    uint32_t slot = g_sta_slot;
    uint32_t scanned = 0u;
    for (; scanned < STA_TOTAL_SLOTS; scanned++) {
        if (flash_range_is_erased(sta_slot_addr(slot), STA_SLOT_BYTES)) break;
        slot = (slot + 1u) % STA_TOTAL_SLOTS;
    }
    *erase = scanned == STA_TOTAL_SLOTS;
    if (*erase) {
        uint32_t page = g_sta_slot / STA_SLOTS_PER_PAGE;
        if (g_sta_have_saved && page == g_sta_saved_slot / STA_SLOTS_PER_PAGE)
            page = (page + 1u) % STA_PAGE_COUNT;
        slot = page * STA_SLOTS_PER_PAGE;
    }
    return slot;
}

bool Flash_AMS_state_read(uint8_t* loaded_ch)
{
    if (!loaded_ch) return false;
    if (g_sta_scanned) {
        *loaded_ch = g_sta_saved_loaded;
        return true;
    }
    if (g_flash_fault || !Flash_background_idle()) return false;
    Flash_clock_guard clock;
    uint8_t best_ch = 0xFFu;
    uint16_t best_seq = 0u;
    uint32_t best_slot = 0u;
    bool have = false;

    for (uint32_t slot = 0u; slot < STA_TOTAL_SLOTS; slot++) {
        const uint32_t a = sta_slot_addr(slot);
        const uint32_t w0 = *(const volatile uint32_t*)(a + 0u);
        const uint32_t w1 = *(const volatile uint32_t*)(a + 4u);
        if ((w0 >> 24) != STA_TAG) continue;
        // 互补校验: w0 ^ w1 必须等于 MAGIC_STA
        if ((w0 ^ w1) != MAGIC_STA) continue;
        const uint16_t seq = (uint16_t)((w0 >> 8) & 0xFFFFu);
        const uint8_t ch = (uint8_t)(w0 & 0xFFu);
        if (ch >= 4u && ch != 0xFFu) continue;
        if (!have || (int16_t)(seq - best_seq) > 0) {
            have = true;
            best_seq = seq;
            best_ch = ch;
            best_slot = slot;
        }
    }

    g_sta_seq = have ? (uint16_t)(best_seq + 1u) : 0u;
    g_sta_saved_slot = (uint16_t)best_slot;
    g_sta_slot = have ? (uint16_t)((best_slot + 1u) % STA_TOTAL_SLOTS) : 0u;
    g_sta_have_saved = have;
    g_sta_saved_loaded = best_ch;
    g_sta_scanned = true;
    *loaded_ch = best_ch;
    return true;
}

/**
 * @brief 写入 AMS 已加载通道状态到环形日志
 *
 * 每条记录 8 字节: w0=编码数据, w1=w0 XOR MAGIC_STA (互补校验)
 * 写入失败时自动擦除当前页并重试。
 *
 * @param loaded_ch 当前已加载的耗材通道 (0-3)
 * @return true 写入成功
 */
bool Flash_AMS_state_write(uint8_t loaded_ch)
{
    if (loaded_ch >= 4u && loaded_ch != 0xFFu) return false;
    if (g_flash_fault || !Flash_background_idle()) return false;
    if (g_sta_have_saved && g_sta_saved_loaded == loaded_ch) return true;
    if (!bus_background_ready()) return false;
    Flash_clock_guard clock;
    bool erase;
    const uint32_t slot = sta_find_slot(&erase);
    Flash_job &job = g_flash_job;
    job = {};
    job.kind = 2u;
    job.channel = loaded_ch;
    job.slot = (uint16_t)slot;
    job.addr = sta_slot_addr(slot);
    job.erase = erase;
    job.halfwords = 4u;
    job.words[0] = (STA_TAG << 24) | ((uint32_t)g_sta_seq << 8) | loaded_ch;
    job.words[1] = job.words[0] ^ MAGIC_STA;
    g_sta_slot = (uint16_t)((slot + 1u) % STA_TOTAL_SLOTS);
    return false;
}

/**
 * @brief 校准数据载荷结构 (48 字节)
 *
 * 存储电机电流传感器的 ADC 校准参数,
 * 4 个通道各自独立。
 */
struct alignas(4) Flash_CAL_payload
{
    float offs[4];
    float vmin[4];
    float vmax[4];
    float dm_none[4];
};

static bool cal_latest(Flash_CAL_payload* out, uint32_t* rsv, uint32_t* addr)
{
    bool have = false;
    for (uint8_t i = 0u; i < 2u; i++) {
        const uint32_t page = i ? CAL_BACKUP_ADDR : FLASH_NVM_CAL_ADDR;
        Flash_CAL_payload p;
        uint16_t got = 0u;
        uint32_t candidate = 0u;
        if (!nvm256_read(page, MAGIC_CAL, 2u, &p, sizeof(p), &got, &candidate) || got != sizeof(p)) continue;
        if (!have || (int16_t)((uint16_t)(candidate >> 16) - (uint16_t)(*rsv >> 16)) > 0) {
            if (out) *out = p;
            *rsv = candidate;
            *addr = page;
            have = true;
        }
    }
    return have;
}

static bool cal_commit(const Flash_CAL_payload& p, uint32_t flags)
{
    if (g_flash_fault || !Flash_background_idle()) return false;
    uint32_t rsv = 0u, addr = CAL_BACKUP_ADDR;
    const bool have = cal_latest(nullptr, &rsv, &addr);
    const uint16_t seq = have ? (uint16_t)((rsv >> 16) + 1u) : 0u;
    return nvm256_write(addr == FLASH_NVM_CAL_ADDR ? CAL_BACKUP_ADDR : FLASH_NVM_CAL_ADDR,
        MAGIC_CAL, 2u, ((uint32_t)seq << 16) | flags, &p, sizeof(p));
}

bool Flash_MC_PULL_cal_write_all(const float offs[4], const float vmin[4], const float vmax[4], const int8_t pol[4], const float dm_none[4], uint8_t valid_mask)
{
    Flash_CAL_payload p;
    memcpy(p.offs, offs, sizeof(p.offs));
    memcpy(p.vmin, vmin, sizeof(p.vmin));
    memcpy(p.vmax, vmax, sizeof(p.vmax));
    memcpy(p.dm_none, dm_none, sizeof(p.dm_none));

    uint32_t rsv = (uint32_t)(valid_mask & 0x0Fu) << 8;
    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (pol && pol[ch] < 0) rsv |= (1u << ch); // 负极性置位
    }

    return cal_commit(p, rsv);
}

bool Flash_MC_PULL_cal_read(float offs[4], float vmin[4], float vmax[4], int8_t pol[4], float dm_none[4], uint8_t* valid_mask)
{
    Flash_CAL_payload p;
    uint32_t rsv = 0u, addr = 0u;
    if (!cal_latest(&p, &rsv, &addr)) return false;
    if (valid_mask) *valid_mask = (uint8_t)((rsv >> 8) & 0x0Fu);

    memcpy(offs, p.offs, sizeof(p.offs));
    memcpy(vmin, p.vmin, sizeof(p.vmin));
    memcpy(vmax, p.vmax, sizeof(p.vmax));
    memcpy(dm_none, p.dm_none, sizeof(p.dm_none));

    // 从 rsv 字段解码极性
    if (pol)
    {
        for (uint8_t ch = 0u; ch < 4u; ch++)
            pol[ch] = (rsv & (1u << ch)) ? -1 : 1;
    }

    return true;
}

/**
 * @brief 清除电机拉动校准数据 (擦除校准页)
 * @return true 擦除成功
 */
bool Flash_MC_PULL_cal_clear(void)
{
    const Flash_CAL_payload p = {};
    return cal_commit(p, 0u);
}

/**
 * @brief 将运动参数写入 Flash
 *
 * 使用通用 NVM256 页格式存储, 魔数为 MAGIC_MOT。
 *
 * @param in 输入数据 (任意结构, 只要不超过 252 字节)
 * @param bytes 数据字节数
 * @return true 写入成功
 */
bool Flash_Motion_write(const void* in, uint16_t bytes)
{
    if (!in || bytes == 0u) return false;
    return nvm256_write(FLASH_NVM_MOTION_ADDR, MAGIC_MOT, VER_1, 0u, in, bytes);
}

/**
 * @brief 从 Flash 读取运动参数
 * @param out 输出缓冲区
 * @param bytes 缓冲区最大长度
 * @return true 读取成功
 */
bool Flash_Motion_read(void* out, uint16_t bytes)
{
    if (!out || bytes == 0u) return false;

    uint16_t got = 0u;
    if (!nvm256_read(FLASH_NVM_MOTION_ADDR, MAGIC_MOT, VER_1,
                     out, bytes, &got, nullptr))
        return false;

    return (got != 0u) && (got <= bytes);
}

/**
 * @brief 清除运动参数 (擦除运动页)
 * @return true 擦除成功
 */
bool Flash_Motion_clear(void)
{
    return flash256_erase(FLASH_NVM_MOTION_ADDR);
}
