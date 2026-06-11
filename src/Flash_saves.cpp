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
#include <string.h>

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
    const uint32_t* p = (const uint32_t*)base_addr;

    for (uint32_t i = 0u; i < (bytes >> 2); i++)
    {
        if (p[i] != FLASH_ERASED_WORD) return false;
    }

    return true;
}

/**
 * @brief 快速编程一页 256 字节 Flash
 *
 * 使用 CH32V203 的快速编程接口 (FLASH_Unlock_Fast / FLASH_ProgramPage_Fast),
 * 比标准编程速度快。流程:
 * 1. 检查地址对齐
 * 2. 比较新旧数据, 相同则跳过
 * 3. 关中断 → 擦除页 → 编程页 → 恢复中断
 * 4. 验证编程结果
 *
 * @param page_addr 页起始地址 (必须 256 字节对齐)
 * @param w 待写入的 64 个 uint32 (共 256 字节)
 * @return true 编程成功且验证通过
 */
static bool flash256_prog(uint32_t page_addr, const uint32_t w[64])
{
    // 地址必须 256 字节对齐
    if (page_addr & (FLASH_NVM256_PAGE_SIZE - 1u)) return false;

    // 新旧数据相同则无需编程, 减少 Flash 磨损
    if (memcmp((const void*)page_addr, (const void*)w, FLASH_NVM256_PAGE_SIZE) == 0)
        return true;

    const uint32_t irq = irq_save_wch();     // 保存中断状态并关中断
    FLASH_Unlock_Fast();                      // 解锁快速编程模式
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR); // 清除标志
    FLASH_ErasePage_Fast(page_addr);          // 快速擦除整页
    FLASH_ProgramPage_Fast(page_addr, (uint32_t*)w); // 快速编程整页
    FLASH_Lock_Fast();                        // 锁定快速编程
    FLASH_Lock();                             // 锁定 Flash
    irq_restore_wch(irq);                     // 恢复中断

    // 编程后验证: 比对写入数据与 Flash 内容
    return (memcmp((const void*)page_addr, (const void*)w, FLASH_NVM256_PAGE_SIZE) == 0);
}

/**
 * @brief 擦除一页 256 字节 Flash (不写入)
 * @param page_addr 页起始地址 (必须 256 字节对齐)
 * @return true 擦除成功且验证通过
 */
static bool flash256_erase(uint32_t page_addr)
{
    if (page_addr & (FLASH_NVM256_PAGE_SIZE - 1u)) return false;

    const uint32_t irq = irq_save_wch();
    FLASH_Unlock_Fast();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage_Fast(page_addr);
    FLASH_Lock_Fast();
    FLASH_Lock();
    irq_restore_wch(irq);

    return flash_range_is_erased(page_addr, FLASH_NVM256_PAGE_SIZE);
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
    if (addr & 3u) return false; // 地址必须 4 字节对齐

    const uint32_t cur = *(const volatile uint32_t*)addr;
    if (cur == data) return true; // 数据已相同, 跳过

    const uint32_t irq = irq_save_wch();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    const FLASH_Status st = FLASH_ProgramWord(addr, data); // 标准字编程
    FLASH_Lock();
    irq_restore_wch(irq);

    return (st == FLASH_COMPLETE) && (*(const volatile uint32_t*)addr == data);
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

    return flash256_prog(page_addr, w);
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
    bool found = false;

    if (first_empty) *first_empty = FIL_SLOTS_PER_PAGE; // 默认: 全满

    for (uint32_t s = 0u; s < FIL_SLOTS_PER_PAGE; s++)
    {
        const uint32_t* p = (const uint32_t*)(base + s * FIL_SLOT_BYTES);

        // 空白槽位: 跳过, 但记录第一个空位
        if (flash_word_is_blank(p[0]))
        {
            if (first_empty && *first_empty == FIL_SLOTS_PER_PAGE)
                *first_empty = s;
            continue;
        }

        // 有效槽位: 更新最新数据
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
    g_fil_first_empty[filament_idx] = 0u;
    memset(&g_fil_last[filament_idx], 0, sizeof(g_fil_last[filament_idx]));
}

/** @brief 状态记录标识字节: 0xA5, 用于区分有效记录和空白 */
static constexpr uint32_t STA_TAG = 0xA5u;
/** @brief 状态记录占用的起始页索引 (相对 NVM 基地址, 页6开始) */
static constexpr uint32_t STA_PAGE_FIRST = 6u;
/** @brief 状态记录占用的总页数 (10 页) */
static constexpr uint32_t STA_PAGE_COUNT = 10u;
/** @brief 单条状态记录大小: 8 字节 (2 个 uint32) */
static constexpr uint32_t STA_SLOT_BYTES = 8u;
/** @brief 每页状态记录条数: 256 / 8 = 32 条 */
static constexpr uint32_t STA_SLOTS_PER_PAGE = (FLASH_NVM256_PAGE_SIZE / STA_SLOT_BYTES);
/** @brief 状态记录总槽位数: 10 * 32 = 320 条 */
static constexpr uint32_t STA_TOTAL_SLOTS = (STA_PAGE_COUNT * STA_SLOTS_PER_PAGE);

/** @brief 当前序列号 (每次写入递增, 用于找到最新记录) */
static uint16_t g_sta_seq = 0u;
/** @brief 下一个可写入的状态槽位索引 */
static uint16_t g_sta_slot = 0u;
/** @brief 是否已保存过至少一条状态记录 */
static uint8_t g_sta_have_saved = 0u;
/** @brief 上次保存的通道号 (用于去重), 0xFF 表示未保存 */
static uint8_t g_sta_saved_loaded = 0xFFu;

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
    g_sta_have_saved = 0u;
    g_sta_saved_loaded = 0xFFu;
}

/**
 * @brief 完全擦除 NVM 4KB Flash 扇区
 *
 * 擦除后验证整段区域是否为空白, 然后重置所有缓存。
 * @return true 擦除成功
 */
bool Flash_NVM_full_clear(void)
{
    const uint32_t irq = irq_save_wch();

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    const FLASH_Status st = FLASH_ErasePage(FLASH_NVM_BASE_ADDR);
    FLASH_Lock();
    irq_restore_wch(irq);

    if (st != FLASH_COMPLETE) return false;
    if (!flash_range_is_erased(FLASH_NVM_BASE_ADDR, FLASH_NVM_TOTAL_SIZE)) return false;

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

/**
 * @brief 初始化 Flash NVM 子系统
 *
 * 使能 CRC 外设时钟, 重置缓存, 并逐个加载 4 个耗材槽位的缓存。
 */
void Flash_saves_init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE); // 使能硬件 CRC 时钟

    flash_runtime_cache_clear();

    for (uint8_t i = 0u; i < 4u; i++)
        fil_cache_load_one(i); // 加载每个耗材槽位的缓存
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

    // 去重: 与缓存中最新数据比较, 相同则跳过
    if (g_fil_have[filament_idx] &&
        memcmp(&g_fil_last[filament_idx], info, sizeof(*info)) == 0)
        return true;

    uint32_t first_empty = g_fil_first_empty[filament_idx];
    const uint32_t base = fil_page_addr(filament_idx);

    // 页已满: 擦除后从头开始
    if (first_empty >= FIL_SLOTS_PER_PAGE)
    {
        if (!flash256_erase(base)) return false;
        first_empty = 0u;
    }

    // 打包并写入
    alignas(4) uint32_t w[FIL_SLOT_WORDS];
    fil_slot_pack(w, info);

    if (!flash_prog_words(base + first_empty * FIL_SLOT_BYTES, w, FIL_SLOT_WORDS))
        return false;

    // 更新缓存
    memcpy(&g_fil_last[filament_idx], info, sizeof(*info));
    g_fil_have[filament_idx] = 1u;
    g_fil_first_empty[filament_idx] =
        (uint8_t)(((first_empty + 1u) < FIL_SLOTS_PER_PAGE) ? (first_empty + 1u) : FIL_SLOTS_PER_PAGE);

    return true;
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

/**
 * @brief 从环形状态日志中读取最新的 AMS 已加载通道
 *
 * 状态记录布局 (每条 8 字节 = 2 个 uint32):
 *   w0 = [TAG:8][SEQ:16][CH:8]  (标识 | 序列号 | 通道号)
 *   w1 = w0 XOR MAGIC_STA       (互补校验, 防止单比特翻转误判)
 *
 * 扫描全部 320 个槽位, 找到序列号最大的有效记录。
 *
 * @param loaded_ch 输出: 已加载的耗材通道 (0-3), 无数据时为 0xFF
 * @return true 读取成功
 */
bool Flash_AMS_state_read(uint8_t* loaded_ch)
{
    if (!loaded_ch) return false;

    uint8_t best_ch = 0xFFu;   // 最佳(最新)通道号
    uint16_t best_seq = 0u;    // 最佳序列号
    uint32_t best_slot = 0u;   // 最佳槽位索引
    uint8_t have = 0u;         // 是否找到有效记录

    for (uint32_t slot = 0u; slot < STA_TOTAL_SLOTS; slot++)
    {
        const uint32_t a = sta_slot_addr(slot);
        const uint32_t w0 = *(const volatile uint32_t*)(a + 0u);
        const uint32_t w1 = *(const volatile uint32_t*)(a + 4u);

        // 跳过全空白槽位
        if (flash_word_is_blank(w0) && flash_word_is_blank(w1)) continue;
        // TAG 字节校验
        if ((w0 >> 24) != STA_TAG) continue;
        // 互补校验: w0 ^ w1 必须等于 MAGIC_STA
        if ((w0 ^ w1) != MAGIC_STA) continue;

        const uint16_t seq = (uint16_t)((w0 >> 8) & 0xFFFFu); // 提取序列号
        const uint8_t ch = (uint8_t)(w0 & 0xFFu);             // 提取通道号

        // 找到序列号更大的记录 (使用有符号比较处理回绕)
        if (!have || (int16_t)(seq - best_seq) > 0)
        {
            have = 1u;
            best_seq = seq;
            best_ch = ch;
            best_slot = slot;
        }
    }

    if (have)
    {
        // 更新写入位置: 序列号+1, 槽位前移
        g_sta_seq = (uint16_t)(best_seq + 1u);
        g_sta_slot = (uint16_t)((best_slot + 1u) % STA_TOTAL_SLOTS);
        g_sta_have_saved = 1u;
        g_sta_saved_loaded = best_ch;
    }
    else
    {
        // 无有效记录: 从头开始
        g_sta_seq = 0u;
        g_sta_slot = 0u;
        g_sta_have_saved = 0u;
        g_sta_saved_loaded = 0xFFu;
    }

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
    // 去重: 通道号未变则跳过
    if (g_sta_have_saved && g_sta_saved_loaded == loaded_ch)
        return true;

    // 编码 w0: [TAG:8][SEQ:16][CH:8]
    const uint16_t seq = g_sta_seq;
    const uint32_t w0 = ((uint32_t)STA_TAG << 24) | ((uint32_t)seq << 8) | (uint32_t)loaded_ch;
    const uint32_t w1 = w0 ^ MAGIC_STA;  // 互补校验值
    const uint32_t slot = (uint32_t)g_sta_slot;
    const uint32_t addr = sta_slot_addr(slot);
    const uint32_t buf[2] = { w0, w1 };

    // 尝试直接编程 (Flash 位只能 1→0)
    if (!flash_prog_words(addr, buf, 2u))
    {
        // 编程失败: 当前页可能已部分使用, 擦除后重试
        const uint32_t page_i = slot / STA_SLOTS_PER_PAGE;
        if (!flash256_erase(sta_page_addr(page_i))) return false;
        if (!flash_prog_words(addr, buf, 2u)) return false;
    }

    // 更新状态
    g_sta_seq = (uint16_t)(seq + 1u);
    g_sta_slot = (uint16_t)((slot + 1u) % STA_TOTAL_SLOTS);
    g_sta_have_saved = 1u;
    g_sta_saved_loaded = loaded_ch;

    return true;
}

/**
 * @brief 校准数据载荷结构 (48 字节)
 *
 * 存储电机电流传感器的 ADC 校准参数,
 * 4 个通道各自独立。
 */
struct alignas(4) Flash_CAL_payload
{
    float offs[4];  /**< 4 通道 ADC 零点偏移量 */
    float vmin[4];  /**< 4 通道 ADC 最小读数对应的物理量 */
    float vmax[4];  /**< 4 通道 ADC 最大读数对应的物理量 */
};

/**
 * @brief 写入电机拉动校准数据到 Flash
 *
 * 将偏移量、最小/最大电压值打包为载荷,
 * 极性信息编码到 NVM256_HDR 的 rsv 字段中 (每位代表一个通道)。
 *
 * @param offs[4] 4通道 ADC 偏移量
 * @param vmin[4] 4通道 ADC 最小电压值
 * @param vmax[4] 4通道 ADC 最大电压值
 * @param pol[4] 4通道极性 (负值编码为 rsv 对应位)
 * @return true 写入成功
 */
bool Flash_MC_PULL_cal_write_all(const float offs[4], const float vmin[4], const float vmax[4], const int8_t pol[4])
{
    Flash_CAL_payload p;
    memcpy(p.offs, offs, sizeof(p.offs));
    memcpy(p.vmin, vmin, sizeof(p.vmin));
    memcpy(p.vmax, vmax, sizeof(p.vmax));

    // 将极性编码到 rsv 字段: 每个通道的极性占 1 位
    uint32_t rsv = 0u;
    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (pol && pol[ch] < 0) rsv |= (1u << ch); // 负极性置位
    }

    return nvm256_write(FLASH_NVM_CAL_ADDR, MAGIC_CAL, VER_1, rsv, &p, (uint16_t)sizeof(p));
}

/**
 * @brief 从 Flash 读取电机拉动校准数据
 * @param offs[4] 输出: 4通道 ADC 偏移量
 * @param vmin[4] 输出: 4通道 ADC 最小电压值
 * @param vmax[4] 输出: 4通道 ADC 最大电压值
 * @param pol[4] 输出: 4通道极性 (1=正, -1=负), 传 NULL 不返回
 * @return true 读取成功
 */
bool Flash_MC_PULL_cal_read(float offs[4], float vmin[4], float vmax[4], int8_t pol[4])
{
    Flash_CAL_payload p;
    uint16_t got = 0u;
    uint32_t rsv = 0u;

    if (!nvm256_read(FLASH_NVM_CAL_ADDR, MAGIC_CAL, VER_1,
                     &p, (uint16_t)sizeof(p), &got, &rsv))
        return false;

    if (got != sizeof(p)) return false; // 载荷长度不匹配

    memcpy(offs, p.offs, sizeof(p.offs));
    memcpy(vmin, p.vmin, sizeof(p.vmin));
    memcpy(vmax, p.vmax, sizeof(p.vmax));

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
    return flash256_erase(FLASH_NVM_CAL_ADDR);
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
