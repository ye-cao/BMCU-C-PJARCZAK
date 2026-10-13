#include "bmcu_link.h"

#if BMCU_LINK_ENABLED

#include "bmcu_link_protocol.h"
#include "bmcu_soft_reset_policy.h"
#include "bmcu_ams_service_watch.h"
#include "_bus_hardware.h"
#include "bambu_bus_ams.h"

#include "Motion_control.h"
#include "ams.h"
#include "ws2812.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_misc.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_usart.h"
#include "core_riscv.h"
#include "hal/time_hw.h"
#include "hal/irq_wch.h"

extern WS2812_class SYS_RGB;

namespace
{
using namespace bmcu_link_protocol;

constexpr uint8_t kMaxDecoded = 64u;
constexpr uint8_t kMaxWire = 66u;
constexpr uint8_t kTxSlots = 8u;
constexpr uint8_t kRxRingSize = 128u;
constexpr uint8_t kSync0 = 0xA5u;
constexpr uint8_t kSync1 = 0x5Au;
constexpr uint8_t kHeaderSize = 5u;
constexpr uint8_t kCrcSize = 2u;
static_assert((kTxSlots & (kTxSlots - 1u)) == 0u, "TX ring must be power of two");
static_assert((kRxRingSize & (kRxRingSize - 1u)) == 0u, "RX ring must be power of two");

struct TxSlot
{
    uint8_t length;
    uint8_t data[kMaxWire];
};

struct FullStatusRecord
{
    uint8_t type;
    uint8_t data[16];
};

struct StatusCache
{
    uint8_t slot;
    uint8_t inserted_mask;
    uint8_t online_mask;
    uint8_t motion[4];
    uint8_t pull_pct[4];
    uint16_t pressure;
    uint8_t led_mode;
    uint8_t control_error;
};

struct PrinterBusCache
{
    uint32_t last_valid_tick;
    uint16_t valid_rx_count;
    uint16_t invalid_rx_count;
    uint16_t tx_count;
    uint16_t tx_drop_count;
    uint8_t online;
    uint8_t last_rx_class;
    uint8_t last_command;
    uint8_t last_outcome;
};

struct PrinterTransactionEventCache
{
    uint32_t last_emit_tick;
    uint8_t command;
    uint8_t outcome;
    uint8_t reason;
    uint8_t valid;
};

struct PrinterAuthCache
{
    uint16_t last_type;
    uint16_t count_040d;
    uint16_t count_040e;
    uint16_t last_payload_length;
    uint32_t last_tick;
    uint8_t last_outcome;
    uint8_t last_reason;
    uint8_t last_response_length;
    uint8_t last_payload_hash;
};

// GLOBAL 1 + CHANNEL 4 + PRINTER_BUS 1 + PRINTER_AUTH 1 + PRINTER_RX 3 +
// PRINTER_TX 2 + AMS_SERVICE 1 + AMS_REGISTRATION 1 + COUNTERS 1 = 15.
#if BMCU_LINK_FULL_STATUS
constexpr uint8_t kMaxFullStatusRecords = 15u;
#else
constexpr uint8_t kMaxFullStatusRecords = 1u;  // 最小化，仅保留基本状态
#endif
constexpr uint8_t kEventSlots = 8u;
static_assert((kEventSlots & (kEventSlots - 1u)) == 0u, "Event ring must be power of two");

TxSlot g_tx[kTxSlots];
uint8_t g_tx_read = 0u;
uint8_t g_tx_write = 0u;
uint8_t g_tx_active = 0u;
uint16_t g_sequence = 0u;
uint32_t g_tx_drop = 0u;

volatile uint8_t g_rx[kRxRingSize];
volatile uint8_t g_rx_read = 0u;
volatile uint8_t g_rx_write = 0u;
volatile uint32_t g_rx_drop = 0u;
uint32_t g_rx_crc_error = 0u;
uint32_t g_rx_frame_error = 0u;
uint8_t g_rx_frame[kMaxDecoded];
uint8_t g_rx_frame_length = 0u;
uint8_t g_rx_expected_length = 0u;
uint8_t g_rx_sync_state = 0u;

uint32_t g_last_led_tick = 0u;
volatile uint32_t g_status_dirty = 0u;
uint8_t g_led_mode = 0u;
uint16_t g_led_remaining_s = 0u;
uint8_t g_led_restore_pending = 0u;
int g_control_error = 0;
bool g_calibration_busy = false;

FullStatusRecord g_full_records[kMaxFullStatusRecords];
uint16_t g_full_snapshot_id = 0u;
uint16_t g_full_sequence = 0u;
uint32_t g_full_tick = 0u;
uint8_t g_full_record_count = 0u;
uint8_t g_full_record_next = 0u;
uint8_t g_full_active = 0u;
#if BMCU_LINK_FULL_STATUS
uint8_t g_full_prefer_record = 0u;
#endif
uint32_t g_full_record_drop = 0u;
FullStatusRecord g_full_record_dummy = {};

StatusCache g_status_cache = {};
PrinterBusCache g_printer_bus = {};
PrinterTransactionEventCache g_printer_transaction_event = {};
uint32_t g_printer_transaction_event_suppressed = 0u;
PrinterAuthCache g_printer_auth = {};
bmcu_ams_service::Watch g_ams_service = {};
uint8_t g_status_cache_valid = 0u;
LogRecord g_events[kEventSlots];
uint8_t g_event_read = 0u;
uint8_t g_event_write = 0u;
uint32_t g_event_drop = 0u;

enum LinkDiag : uint8_t
{
    kDiagOff = 0u,
    kDiagInit = 1u,
    kDiagTxPending = 2u,
    kDiagTxOk = 3u,
    kDiagTxTimeout = 4u,
};
uint8_t g_link_diag = kDiagOff;
uint8_t g_tx_fault = 0u;
uint32_t g_tx_fault_cooldown_tick = 0u;
uint32_t g_tx_fault_count = 0u;
uint32_t g_tx_started_tick = 0u;
uint32_t g_diag_until_tick = 0u;
uint32_t g_rx_diag_until_tick = 0u;

enum SoftResetPhase : uint8_t { kResetIdle = 0u, kResetDrain = 1u, kResetSettle = 2u };
uint8_t g_reset_phase = kResetIdle;
uint32_t g_reset_operation_id = 0u;
uint32_t g_last_reset_operation_id = 0u;
uint32_t g_reset_deadline_tick = 0u;
uint32_t g_reset_settle_tick = 0u;
uint32_t g_last_printer_motion_tick = 0u;
uint8_t g_reset_request_reason = 0u;
uint8_t g_have_last_reset_operation = 0u;
uint8_t g_have_printer_motion_tick = 0u;


uint16_t crc16(const uint8_t* data, uint8_t length)
{
    // CCITT-FALSE nibble table: two small, branch-free rounds per byte.
    // A 16-entry table avoids the flash/bus footprint of a 256-entry table
    // while replacing the former eight inner-loop branches.
    static constexpr uint16_t table[16] = {
        0x0000u, 0x1021u, 0x2042u, 0x3063u,
        0x4084u, 0x50A5u, 0x60C6u, 0x70E7u,
        0x8108u, 0x9129u, 0xA14Au, 0xB16Bu,
        0xC18Cu, 0xD1ADu, 0xE1CEu, 0xF1EFu,
    };
    uint16_t crc = 0xFFFFu;
    while (length--)
    {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        crc = static_cast<uint16_t>((crc << 4) ^ table[crc >> 12]);
        crc = static_cast<uint16_t>((crc << 4) ^ table[crc >> 12]);
    }
    return crc;
}

void tx_start_if_idle()
{
    if (g_tx_active || g_tx_read == g_tx_write) return;
    TxSlot& slot = g_tx[g_tx_read];
    DMA1_Channel2->CFGR &= ~static_cast<uint32_t>(DMA_CFGR2_EN);
    DMA1->INTFCR = DMA1_FLAG_TC2 | DMA1_FLAG_GL2;
    USART3->STATR = static_cast<uint16_t>(~USART_STATR_TC);
    DMA1_Channel2->MADDR = reinterpret_cast<uint32_t>(slot.data);
    DMA1_Channel2->CNTR = slot.length;
    DMA1_Channel2->CFGR |= DMA_CFGR2_EN;
    g_tx_active = 1u;
    g_tx_started_tick = time_ticks32();
    g_link_diag = kDiagTxPending;
}

void tx_finish_if_complete()
{
    if (!g_tx_active) return;
    if ((DMA1->INTFR & DMA1_FLAG_TC2) == 0u) return;
    if ((USART3->STATR & USART_STATR_TC) == 0u) return;
    DMA1_Channel2->CFGR &= ~static_cast<uint32_t>(DMA_CFGR2_EN);
    DMA1->INTFCR = DMA1_FLAG_TC2 | DMA1_FLAG_GL2;
    g_tx_active = 0u;
    g_tx_read = static_cast<uint8_t>((g_tx_read + 1u) & (kTxSlots - 1u));
    g_link_diag = kDiagTxOk;
    g_diag_until_tick = time_ticks32() + time_hw_tpms * 5000u;
    tx_start_if_idle();
}

__attribute__((noinline, cold)) void tx_abort_cold()
{
    DMA1_Channel2->CFGR &= ~static_cast<uint32_t>(DMA_CFGR2_EN);
    DMA1->INTFCR = DMA1_FLAG_GL2 | DMA1_FLAG_TC2 | DMA1_FLAG_HT2 | DMA1_FLAG_TE2;
    g_tx_active = 0u;
    g_tx_fault = 1u;
    g_tx_read = g_tx_write;
    ++g_tx_drop;
    ++g_tx_fault_count;
    // Recoverable fault: re-arm after a cooldown instead of latching forever.
    // Every abort restarts the cooldown, so repeated faults cannot busy-loop.
    g_tx_fault_cooldown_tick = time_ticks32() + time_hw_tpms * 150u;
    g_link_diag = kDiagTxTimeout;
}

void tx_fail_if_needed(uint32_t now)
{
    if (!g_tx_active) return;
    const uint32_t timeout_ticks = time_hw_tpms * 250u;
    const bool timed_out = timeout_ticks != 0u &&
                           static_cast<uint32_t>(now - g_tx_started_tick) >= timeout_ticks;
    const bool transfer_error = (DMA1->INTFR & DMA1_FLAG_TE2) != 0u;
    if (!timed_out && !transfer_error) return;
    tx_abort_cold();
}

// Main-loop driven, same context as tx_fail_if_needed/tx_finish_if_complete.
// Once the abort cooldown expires, fully re-arm the TX path so the BMCU can
// transmit again after a transient DMA timeout/transfer error.
void tx_recover_if_ready(uint32_t now)
{
    if (!g_tx_fault) return;
    if (time_diff32(now, g_tx_fault_cooldown_tick) < 0) return;

    // Clear pending DMA channel-2 flags and make sure the channel is disabled.
    DMA1_Channel2->CFGR &= ~static_cast<uint32_t>(DMA_CFGR2_EN);
    DMA1->INTFCR = DMA1_FLAG_GL2 | DMA1_FLAG_TC2 | DMA1_FLAG_HT2 | DMA1_FLAG_TE2;
    // Clear the USART transmit-complete flag so the next frame starts clean.
    USART3->STATR = static_cast<uint16_t>(~USART_STATR_TC);
    g_tx_active = 0u;
    g_tx_fault = 0u;
    // Queue was drained on abort; resume from the (now empty) ring.
    tx_start_if_idle();
}

bool tx_has_capacity(uint8_t reserved_slots)
{
    const uint8_t used = static_cast<uint8_t>((g_tx_write - g_tx_read) & (kTxSlots - 1u));
    return used < static_cast<uint8_t>((kTxSlots - 1u) - reserved_slots);
}

uint8_t* reserve_payload(uint8_t kind, uint16_t sequence, uint8_t payload_length)
{
    if (g_tx_fault)
    {
        ++g_tx_drop;
        return nullptr;
    }
    if (payload_length > static_cast<uint8_t>(kMaxDecoded - kHeaderSize - kCrcSize))
        return nullptr;
    const uint8_t next = static_cast<uint8_t>((g_tx_write + 1u) & (kTxSlots - 1u));
    if (next == g_tx_read)
    {
        ++g_tx_drop;
        return nullptr;
    }

    TxSlot& slot = g_tx[g_tx_write];
    slot.data[0] = kSync0;
    slot.data[1] = kSync1;
    slot.data[2] = VERSION;
    slot.data[3] = kind;
    slot.data[4] = static_cast<uint8_t>(sequence);
    slot.data[5] = static_cast<uint8_t>(sequence >> 8);
    slot.data[6] = payload_length;
    return &slot.data[7];
}

void commit_payload(uint8_t payload_length)
{
    TxSlot& slot = g_tx[g_tx_write];
    const uint8_t crc_length = static_cast<uint8_t>(kHeaderSize + payload_length);
    const uint16_t crc = crc16(&slot.data[2], crc_length);
    slot.data[7u + payload_length] = static_cast<uint8_t>(crc);
    slot.data[8u + payload_length] = static_cast<uint8_t>(crc >> 8);
    slot.length = static_cast<uint8_t>(payload_length + 9u);
    g_tx_write = static_cast<uint8_t>((g_tx_write + 1u) & (kTxSlots - 1u));
    tx_start_if_idle();
}

void put16(uint8_t* output, uint16_t value)
{
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

uint32_t get32(const uint8_t* input)
{
    return static_cast<uint32_t>(input[0]) |
           (static_cast<uint32_t>(input[1]) << 8) |
           (static_cast<uint32_t>(input[2]) << 16) |
           (static_cast<uint32_t>(input[3]) << 24);
}

void put32(uint8_t* output, uint32_t value)
{
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
    output[2] = static_cast<uint8_t>(value >> 16);
    output[3] = static_cast<uint8_t>(value >> 24);
}

void send_hello()
{
    uint8_t* payload = reserve_payload(KIND_HELLO, g_sequence, 9u);
    if (payload == nullptr) return;
    payload[0] = VERSION;
    put16(&payload[1], CAP_STATUS_EVENTS | CAP_LED_OVERRIDE | CAP_PING_PONG |
                          CAP_RAW_HW_TICK | CAP_PRINTER_TRACE | CAP_FULL_STATUS |
                          CAP_SOFT_RESET);
    payload[3] = 1u;
    payload[4] = 1u;
    put32(&payload[5], time_hw_tpus * 1000000u);
    commit_payload(9u);
    ++g_sequence;
}

void push_log_record(const LogRecord& record)
{
    const uint8_t next = static_cast<uint8_t>((g_event_write + 1u) & (kEventSlots - 1u));
    if (next == g_event_read)
        ++g_event_drop;
    else
    {
        g_events[g_event_write] = record;
        g_event_write = next;
    }
}

void push_state_event(StateField field, uint8_t slot, uint16_t previous_value, uint16_t value,
                      RecordSource source, RecordSeverity severity)
{
    if (previous_value == value) return;
    LogRecord record = {};
    record.header.hw_tick32 = time_ticks32();
    record.header.type = RECORD_STATE_CHANGE;
    record.header.severity = severity;
    record.header.source = source;
    record.header.payload_length = sizeof(LogStateChangePayload);
    record.payload.state_change.field = static_cast<uint8_t>(field);
    record.payload.state_change.slot = slot;
    record.payload.state_change.previous_value = previous_value;
    record.payload.state_change.value = value;

    push_log_record(record);
}

uint8_t inserted_mask()
{
    uint8_t value = 0u;
    for (uint8_t ch = 0u; ch < 4u; ++ch)
        if (filament_channel_inserted[ch]) value |= static_cast<uint8_t>(1u << ch);
    return value;
}

uint8_t online_mask(const _ams& state)
{
    uint8_t value = 0u;
    for (uint8_t ch = 0u; ch < 4u; ++ch)
        if (state.filament[ch].online) value |= static_cast<uint8_t>(1u << ch);
    return value;
}

void set_cached_u8(uint8_t& cached, uint8_t value, StateField field, uint8_t slot,
                   RecordSource source, RecordSeverity severity, bool emit)
{
    const uint8_t previous = cached;
    cached = value;
    if (emit) push_state_event(field, slot, previous, value, source, severity);
}

void set_cached_u16(uint16_t& cached, uint16_t value, StateField field,
                    RecordSource source, RecordSeverity severity, bool emit)
{
    const uint16_t previous = cached;
    cached = value;
    if (emit) push_state_event(field, 0xFFu, previous, value, source, severity);
}

void apply_status_changes(uint32_t reasons, bool emit_events)
{
    const _ams& state = ams[bambubus_local_ams_index()];
    const bool emit = emit_events && g_status_cache_valid;
    const bool all = reasons == BMCU_STATUS_CHANGE_ALL;

    if (all || (reasons & BMCU_STATUS_CHANGE_SLOT))
        set_cached_u8(g_status_cache.slot, state.now_filament_num, STATE_FIELD_SLOT, 0xFFu,
                      SOURCE_MOTION, SEVERITY_INFO, emit);
    if (all || (reasons & BMCU_STATUS_CHANGE_INSERTED))
        set_cached_u8(g_status_cache.inserted_mask, inserted_mask(), STATE_FIELD_INSERTED_MASK,
                      0xFFu, SOURCE_SENSOR, SEVERITY_NOTICE, emit);
    if (all || (reasons & BMCU_STATUS_CHANGE_ONLINE))
        set_cached_u8(g_status_cache.online_mask, online_mask(state), STATE_FIELD_ONLINE_MASK,
                      0xFFu, SOURCE_SENSOR, SEVERITY_NOTICE, emit);
    if (all || (reasons & BMCU_STATUS_CHANGE_MOTION))
    {
        for (uint8_t ch = 0u; ch < 4u; ++ch)
            set_cached_u8(g_status_cache.motion[ch],
                          static_cast<uint8_t>(state.filament[ch].motion),
                          STATE_FIELD_MOTION, ch, SOURCE_MOTION, SEVERITY_INFO, emit);
    }
    if (all || (reasons & BMCU_STATUS_CHANGE_PRESSURE))
        set_cached_u16(g_status_cache.pressure, state.pressure, STATE_FIELD_PRESSURE,
                       SOURCE_SENSOR, SEVERITY_INFO, emit);
    if (all || (reasons & BMCU_STATUS_CHANGE_LED))
        set_cached_u8(g_status_cache.led_mode, g_led_mode, STATE_FIELD_LED_MODE, 0xFFu,
                      SOURCE_MANAGEMENT, SEVERITY_INFO, emit);
    if (all || (reasons & BMCU_STATUS_CHANGE_ERROR))
        set_cached_u8(g_status_cache.control_error, g_control_error ? 1u : 0u,
                      STATE_FIELD_CONTROL_ERROR, 0xFFu, SOURCE_SAFETY,
                      g_control_error ? SEVERITY_ERROR : SEVERITY_NOTICE, emit);

    if (all || (reasons & (BMCU_STATUS_CHANGE_PRESSURE | BMCU_STATUS_CHANGE_MOTION)))
        __builtin_memcpy(g_status_cache.pull_pct, MC_PULL_pct, sizeof(g_status_cache.pull_pct));
    g_status_cache_valid = 1u;
}
void build_status_payload(uint8_t payload[27])
{
    if (!g_status_cache_valid) apply_status_changes(BMCU_STATUS_CHANGE_ALL, false);
    put32(&payload[0], time_ticks32());
    put16(&payload[4], static_cast<uint16_t>(g_tx_drop));
    put16(&payload[6], static_cast<uint16_t>(g_rx_drop));
    put16(&payload[8], static_cast<uint16_t>(g_rx_crc_error));
    put16(&payload[10], static_cast<uint16_t>(g_rx_frame_error));
    payload[12] = g_status_cache.slot;
    payload[13] = g_status_cache.inserted_mask;
    payload[14] = g_status_cache.online_mask;
    for (uint8_t ch = 0u; ch < 4u; ++ch)
    {
        payload[15u + ch] = g_status_cache.motion[ch];
        payload[19u + ch] = g_status_cache.pull_pct[ch];
    }
    put16(&payload[23], g_status_cache.pressure);
    payload[25] = g_status_cache.led_mode;
    payload[26] = g_status_cache.control_error;
}

FullStatusRecord& append_full_record(uint8_t type)
{
    if (g_full_record_count >= kMaxFullStatusRecords)
    {
        // Never write past the array; hand back a throwaway slot that is not
        // counted and therefore never transmitted.
        ++g_full_record_drop;
        g_full_record_dummy.type = type;
        return g_full_record_dummy;
    }
    FullStatusRecord& record = g_full_records[g_full_record_count++];
    record.type = type;
    return record;
}

#if BMCU_LINK_FULL_STATUS
void capture_full_status(uint8_t section_mask, uint8_t channel_mask, uint16_t sequence)
{
    apply_status_changes(BMCU_STATUS_CHANGE_ALL, false);
    const _ams& state = ams[bambubus_local_ams_index()];
    g_full_record_count = 0u;
    g_full_record_next = 0u;
    g_full_sequence = sequence;
    g_full_tick = time_ticks32();
    ++g_full_snapshot_id;

    if (section_mask & FULL_SECTION_GLOBAL)
    {
        FullStatusRecord& record = append_full_record(FULL_RECORD_GLOBAL);
        record.data[0] = g_status_cache.slot;
        record.data[1] = g_status_cache.inserted_mask;
        record.data[2] = g_status_cache.online_mask;
        record.data[3] = g_status_cache.control_error;
        for (uint8_t ch = 0u; ch < 4u; ++ch)
        {
            record.data[4u + ch] = g_status_cache.motion[ch];
            record.data[8u + ch] = g_status_cache.pull_pct[ch];
        }
        put16(&record.data[12], g_status_cache.pressure);
        record.data[14] = g_status_cache.led_mode;
        record.data[15] = 0u;
    }

    if (section_mask & FULL_SECTION_CHANNELS)
    {
        for (uint8_t ch = 0u; ch < 4u; ++ch)
        {
            if ((channel_mask & static_cast<uint8_t>(1u << ch)) == 0u) continue;
            FullStatusRecord& record = append_full_record(FULL_RECORD_CHANNEL);
            record.data[0] = ch;
            record.data[1] = static_cast<uint8_t>(state.filament[ch].motion);
            record.data[2] = filament_channel_inserted[ch] ? 1u : 0u;
            record.data[3] = state.filament[ch].online ? 1u : 0u;
            record.data[4] = MC_PULL_pct[ch];
            MotionControlChannelTelemetry telemetry = {};
            const bool have_telemetry = Motion_control_get_channel_telemetry(ch, &telemetry);
            if (!have_telemetry) record.data[5] = SENSOR_UNKNOWN;
            else if (telemetry.sensor_good) record.data[5] = SENSOR_VALID;
            else if (telemetry.sensor_online) record.data[5] = SENSOR_FAULT;
            else record.data[5] = SENSOR_OFFLINE;
            uint16_t flags = 0u;
            if (filament_channel_inserted[ch]) flags |= 1u << 0;
            if (state.filament[ch].online) flags |= 1u << 1;
            if (telemetry.sensor_online) flags |= 1u << 2;
            if (telemetry.sensor_good) flags |= 1u << 3;
            if (telemetry.motion_fault != MOTION_FAULT_NONE) flags |= 1u << 4;
            put16(&record.data[6], flags);
            put16(&record.data[8], telemetry.raw_angle);
            put16(&record.data[10], static_cast<uint16_t>(telemetry.position_delta));
            put16(&record.data[12], static_cast<uint16_t>(telemetry.motor_pwm));
            record.data[14] = telemetry.motion_fault;
            record.data[15] = static_cast<uint8_t>(0x80u | telemetry.controller_motion);
        }
    }

    if (section_mask & FULL_SECTION_PRINTER_BUS)
    {
        FullStatusRecord& record = append_full_record(FULL_RECORD_PRINTER_BUS);
        record.data[0] = g_printer_bus.online;
        record.data[1] = g_printer_bus.last_rx_class;
        record.data[2] = g_printer_bus.last_command;
        record.data[3] = g_printer_bus.last_outcome;
        put16(&record.data[4], g_printer_bus.valid_rx_count);
        put16(&record.data[6], g_printer_bus.invalid_rx_count);
        put16(&record.data[8], g_printer_bus.tx_count);
        put16(&record.data[10], g_printer_bus.tx_drop_count);
        const uint32_t age = g_printer_bus.valid_rx_count == 0u
            ? 0xFFFFFFFFu : static_cast<uint32_t>(g_full_tick - g_printer_bus.last_valid_tick);
        put32(&record.data[12], age);

        FullStatusRecord& auth = append_full_record(FULL_RECORD_PRINTER_AUTH);
        put16(&auth.data[0], g_printer_auth.last_type);
        put16(&auth.data[2], g_printer_auth.count_040d);
        put16(&auth.data[4], g_printer_auth.count_040e);
        put16(&auth.data[6], g_printer_auth.last_payload_length);
        put32(&auth.data[8], g_printer_auth.last_tick);
        auth.data[12] = g_printer_auth.last_outcome;
        auth.data[13] = g_printer_auth.last_reason;
        auth.data[14] = g_printer_auth.last_response_length;
        auth.data[15] = g_printer_auth.last_payload_hash;

        FullStatusRecord& rx_core = append_full_record(FULL_RECORD_PRINTER_RX_CORE);
        put32(&rx_core.data[0], bus_port_to_host.rx_metrics.rx_bytes);
        put32(&rx_core.data[4], bus_port_to_host.rx_metrics.rx_frames_valid);
        put32(&rx_core.data[8], bus_port_to_host.rx_metrics.rx_bad_length);
        put32(&rx_core.data[12], bus_port_to_host.rx_metrics.rx_header_crc_error);

        FullStatusRecord& rx_loss = append_full_record(FULL_RECORD_PRINTER_RX_LOSS);
        put32(&rx_loss.data[0], bus_port_to_host.rx_metrics.rx_resync_bytes);
        put32(&rx_loss.data[4], bus_port_to_host.rx_metrics.rx_publish_drop);
        put32(&rx_loss.data[8], bus_port_to_host.rx_metrics.rx_dma_error);
        put32(&rx_loss.data[12], bus_port_to_host.rx_metrics.rx_usart_overrun);

        FullStatusRecord& rx_dma = append_full_record(FULL_RECORD_PRINTER_RX_DMA);
        put32(&rx_dma.data[0], bus_port_to_host.rx_metrics.rx_dma_overrun);
        put32(&rx_dma.data[4], bus_port_to_host.rx_metrics.rx_dma_wrap);
        put32(&rx_dma.data[8], bus_port_to_host.rx_metrics.rx_dma_max_pending);
        put32(&rx_dma.data[12], bus_port_to_host.rx_metrics.rx_compat_copy);

        FullStatusRecord& tx_core = append_full_record(FULL_RECORD_PRINTER_TX_CORE);
        put32(&tx_core.data[0], bus_port_to_host.tx_metrics.tx_started);
        put32(&tx_core.data[4], bus_port_to_host.tx_metrics.tx_completed);
        put32(&tx_core.data[8], bus_port_to_host.tx_metrics.tx_response_busy);
        put32(&tx_core.data[12], bus_port_to_host.tx_metrics.tx_response_missing);

        FullStatusRecord& tx_fault = append_full_record(FULL_RECORD_PRINTER_TX_FAULT);
        put32(&tx_fault.data[0], bus_port_to_host.tx_metrics.tx_invalid_length);
        put32(&tx_fault.data[4], bus_port_to_host.tx_metrics.tx_dma_error);
        put32(&tx_fault.data[8], bus_port_to_host.tx_metrics.tx_timeout);
        put32(&tx_fault.data[12], bus_port_to_host.tx_metrics.tx_no_response_expected);

        bmcu_ams_service::poll(g_ams_service, g_full_tick, time_hw_tpms, nullptr);

        FullStatusRecord& ams_service = append_full_record(FULL_RECORD_AMS_SERVICE);
        put32(&ams_service.data[0], g_ams_service.gap_now_ms);
        put32(&ams_service.data[4], g_ams_service.gap_max_ms);
        put32(&ams_service.data[8], g_ams_service.gap_max_since_confirm_ms);
        put32(&ams_service.data[12], g_ams_service.ms_since_confirm);

        FullStatusRecord& ams_reg = append_full_record(FULL_RECORD_AMS_REGISTRATION);
        put16(&ams_reg.data[0], g_ams_service.service_count[bmcu_ams_service::kServiceMotion]);
        put16(&ams_reg.data[2], g_ams_service.service_count[bmcu_ams_service::kServiceStuMotion]);
        put16(&ams_reg.data[4], g_ams_service.service_count[bmcu_ams_service::kServiceMcOnline]);
        put16(&ams_reg.data[6], g_ams_service.registration_query_count);
        put16(&ams_reg.data[8], g_ams_service.would_reoffer_count);
        put16(&ams_reg.data[10], g_ams_service.confirm_count);
        put16(&ams_reg.data[12], g_ams_service.reset_count);
        uint8_t ams_flags = 0u;
        if (g_ams_service.registered) ams_flags |= 1u << 0;
        if (g_ams_service.confirm_seen) ams_flags |= 1u << 1;
        if (bmcu_ams_service::service_stale(g_ams_service)) ams_flags |= 1u << 2;
        if (bmcu_ams_service::reoffer_armed(g_ams_service)) ams_flags |= 1u << 3;
        if (g_ams_service.have_service) ams_flags |= 1u << 4;
        ams_reg.data[14] = ams_flags;
        ams_reg.data[15] = 0u;
    }

    if (section_mask & FULL_SECTION_COUNTERS)
    {
        FullStatusRecord& record = append_full_record(FULL_RECORD_COUNTERS);
        put32(&record.data[0], g_tx_drop + g_event_drop);
        put32(&record.data[4], g_rx_drop);
        put32(&record.data[8], g_rx_crc_error);
        put32(&record.data[12], g_rx_frame_error);
    }

    g_full_active = g_full_record_count != 0u ? 1u : 0u;
}

bool send_next_full_status_record()
{
    if (!g_full_active || !tx_has_capacity(1u)) return false;

    const FullStatusRecord& record = g_full_records[g_full_record_next];
    uint8_t* payload = reserve_payload(KIND_FULL_STATUS_RECORD, g_full_sequence, 26u);
    if (payload == nullptr) return false;
    put16(&payload[0], g_full_snapshot_id);
    payload[2] = g_full_record_next;
    payload[3] = g_full_record_count;
    payload[4] = record.type;
    payload[5] = 0u;
    put32(&payload[6], g_full_tick);
    __builtin_memcpy(&payload[10], record.data, sizeof(record.data));

    commit_payload(26u);
    if (++g_full_record_next >= g_full_record_count) g_full_active = 0u;
    return true;
}
#else
/* BMCU_LINK_FULL_STATUS=0: 简化版，仅返回全局状态 */
static void capture_full_status(uint8_t, uint8_t, uint16_t sequence)
{
    apply_status_changes(BMCU_STATUS_CHANGE_ALL, false);
    g_full_record_count = 0u;
    g_full_record_next = 0u;
    g_full_sequence = sequence;
    g_full_tick = time_ticks32();
    ++g_full_snapshot_id;

    FullStatusRecord& record = append_full_record(FULL_RECORD_GLOBAL);
    record.data[0] = g_status_cache.slot;
    record.data[1] = g_status_cache.inserted_mask;
    record.data[2] = g_status_cache.online_mask;
    record.data[3] = g_status_cache.control_error;
    for (uint8_t ch = 0u; ch < 4u; ++ch)
    {
        record.data[4u + ch] = g_status_cache.motion[ch];
        record.data[8u + ch] = g_status_cache.pull_pct[ch];
    }
    put16(&record.data[12], g_status_cache.pressure);
    record.data[14] = g_status_cache.led_mode;
    record.data[15] = 0u;

    g_full_active = g_full_record_count != 0u ? 1u : 0u;
}

static bool send_next_full_status_record()
{
    if (!g_full_active || !tx_has_capacity(1u)) return false;

    const FullStatusRecord& record = g_full_records[g_full_record_next];
    uint8_t* payload = reserve_payload(KIND_FULL_STATUS_RECORD, g_full_sequence, 26u);
    if (payload == nullptr) return false;
    put16(&payload[0], g_full_snapshot_id);
    payload[2] = g_full_record_next;
    payload[3] = g_full_record_count;
    payload[4] = record.type;
    payload[5] = 0u;
    put32(&payload[6], g_full_tick);
    __builtin_memcpy(&payload[10], record.data, sizeof(record.data));

    commit_payload(26u);
    if (++g_full_record_next >= g_full_record_count) g_full_active = 0u;
    return true;
}
#endif /* BMCU_LINK_FULL_STATUS */

uint32_t take_status_changes()
{
    const uint32_t pending = g_status_dirty;
    g_status_dirty = 0u;
    return pending;
}

void restore_status_changes(uint32_t reasons)
{
    g_status_dirty |= reasons;
}

bool send_status(uint16_t sequence)
{
    if (!tx_has_capacity(0u)) return false;
    uint8_t* payload = reserve_payload(KIND_STATUS, sequence, 27u);
    if (payload == nullptr) return false;
    build_status_payload(payload);
    commit_payload(27u);
    return true;
}

bool send_next_event()
{
    if (g_event_read == g_event_write || !tx_has_capacity(0u)) return false;
    const LogRecord& record = g_events[g_event_read];
    uint8_t* payload = reserve_payload(KIND_EVENT, g_sequence, sizeof(LogRecord));
    if (payload == nullptr) return false;
    __builtin_memcpy(payload, &record, sizeof(record));
    commit_payload(sizeof(LogRecord));
    ++g_sequence;
    g_event_read = static_cast<uint8_t>((g_event_read + 1u) & (kEventSlots - 1u));
    return true;
}

bool printer_reset_quiescent(uint32_t now)
{
    const uint32_t quiet_ticks = time_hw_tpms * 5000u;
    if (!Motion_control_is_reset_safe()) return false;
    if (!bus_port_to_host.quiet_for_us(5000000u)) return false;
    if (g_have_printer_motion_tick != 0u && quiet_ticks != 0u &&
        static_cast<uint32_t>(now - g_last_printer_motion_tick) < quiet_ticks)
        return false;
    return true;
}

void push_reset_state_event(uint8_t state, uint8_t cancel_reason)
{
    LogRecord record = {};
    record.header.hw_tick32 = time_ticks32();
    record.header.type = RECORD_RESET_STATE;
    record.header.severity = state == RESET_CANCELLED ? SEVERITY_WARNING : SEVERITY_NOTICE;
    record.header.source = SOURCE_SYSTEM;
    record.header.payload_length = sizeof(LogResetStatePayload);
    record.payload.reset_state.operation_id = g_reset_operation_id;
    record.payload.reset_state.state = state;
    record.payload.reset_state.request_reason = g_reset_request_reason;
    record.payload.reset_state.cancel_reason = cancel_reason;
    push_log_record(record);
}

void cancel_soft_reset(uint8_t reason)
{
    if (g_reset_phase == kResetIdle) return;
    g_reset_phase = kResetIdle;
    push_reset_state_event(RESET_CANCELLED, reason);
}

void service_soft_reset(uint32_t now)
{
    if (g_reset_phase == kResetIdle) return;
    if (g_tx_fault != 0u)
    {
        cancel_soft_reset(RESET_CANCEL_LINK_TX_FAULT);
        return;
    }
    if (time_diff32(now, g_reset_deadline_tick) >= 0)
    {
        cancel_soft_reset(RESET_CANCEL_EXPIRED);
        return;
    }
    if (g_calibration_busy || g_full_active || !printer_reset_quiescent(now))
    {
        cancel_soft_reset(RESET_CANCEL_SAFETY_CHANGED);
        return;
    }

    if (g_reset_phase == kResetDrain)
    {
        if (g_event_read != g_event_write)
        {
            (void)send_next_event();
            return;
        }
        if (g_tx_active || g_tx_read != g_tx_write ||
            (USART3->STATR & USART_STATR_TC) == 0u)
            return;

        for (uint8_t channel = 0u; channel < 4u; ++channel)
            Motion_control_set_PWM(channel, 0);
        g_reset_settle_tick = now + time_hw_tpms * 20u;
        g_reset_phase = kResetSettle;
        return;
    }

    if (time_diff32(now, g_reset_settle_tick) >= 0)
        NVIC_SystemReset();
}

bool send_ack(uint16_t sequence, uint8_t request_kind, AckResult result)
{
    uint8_t* payload = reserve_payload(KIND_ACK, sequence, 2u);
    if (payload == nullptr) return false;
    payload[0] = request_kind;
    payload[1] = static_cast<uint8_t>(result);
    commit_payload(2u);
    return true;
}

void send_pong(uint16_t sequence, const uint8_t token[4])
{
    uint8_t* payload = reserve_payload(KIND_PONG, sequence, 8u);
    if (payload == nullptr) return;
    for (uint8_t i = 0u; i < 4u; ++i) payload[i] = token[i];
    put32(&payload[4], time_ticks32());
    commit_payload(8u);
}

void handle_frame(const uint8_t* raw, uint8_t length)
{
    if (length < 7u)
    {
        ++g_rx_frame_error;
        return;
    }
    const uint8_t payload_length = raw[4];
    if (raw[0] != VERSION || static_cast<uint16_t>(payload_length) + 7u != length)
    {
        ++g_rx_frame_error;
        return;
    }
    const uint16_t expected = static_cast<uint16_t>(raw[length - 2u]) |
                              (static_cast<uint16_t>(raw[length - 1u]) << 8);
    if (crc16(raw, static_cast<uint8_t>(length - 2u)) != expected)
    {
        ++g_rx_crc_error;
        return;
    }

    g_rx_diag_until_tick = time_ticks32() + time_hw_tpms * 5000u;

    const uint8_t kind = raw[1];
    const uint16_t sequence = static_cast<uint16_t>(raw[2]) |
                              (static_cast<uint16_t>(raw[3]) << 8);
    const uint8_t* payload = &raw[5];

    switch (kind)
    {
    case KIND_GET_STATUS:
        if (payload_length != 0u)
        {
            send_ack(sequence, kind, ACK_BAD_VALUE);
        }
        else
        {
            const uint32_t pending = take_status_changes();
            apply_status_changes(BMCU_STATUS_CHANGE_ALL, false);
            if (!send_status(sequence)) restore_status_changes(pending);
        }
        break;

    case KIND_GET_FULL_STATUS:
        if (payload_length != 2u || payload[0] == 0u ||
            (payload[0] & static_cast<uint8_t>(~FULL_SECTION_ALL)) != 0u ||
            (payload[1] & 0xF0u) != 0u ||
            (payload[0] == FULL_SECTION_CHANNELS && payload[1] == 0u))
        {
            send_ack(sequence, kind, ACK_BAD_VALUE);
        }
        else if (g_calibration_busy || g_full_active)
        {
            send_ack(sequence, kind, ACK_BUSY);
        }
        else
        {
            capture_full_status(payload[0], payload[1], sequence);
        }
        break;

    case KIND_PING:
        if (payload_length != 4u) send_ack(sequence, kind, ACK_BAD_VALUE);
        else send_pong(sequence, payload);
        break;

    case KIND_SET_LED_MODE:
        if (payload_length != 3u || payload[0] > 3u)
        {
            send_ack(sequence, kind, ACK_BAD_VALUE);
            break;
        }
        {
            const uint16_t timeout = static_cast<uint16_t>(payload[1]) |
                                     (static_cast<uint16_t>(payload[2]) << 8);
            if (timeout == 0u || payload[0] == 0u)
            {
                g_led_mode = 0u;
                g_led_remaining_s = 0u;
                g_led_restore_pending = 1u;
            }
            else
            {
                g_led_mode = payload[0];
                g_led_remaining_s = timeout;
            }
            bmcu_link_status_changed(BMCU_STATUS_CHANGE_LED);
            send_ack(sequence, kind, ACK_OK);
        }
        break;

    case KIND_REQUEST_SOFT_RESET:
        if (payload_length != 8u)
        {
            send_ack(sequence, kind, ACK_BAD_VALUE);
            break;
        }
        {
            bmcu_soft_reset::Request request = {};
            request.operation_id = get32(&payload[0]);
            request.reason = payload[4];
            request.flags = payload[5];
            request.ttl_ms = static_cast<uint16_t>(payload[6]) |
                             (static_cast<uint16_t>(payload[7]) << 8);

            const bmcu_soft_reset::SafetyState safety = {
                g_reset_phase != kResetIdle,
                g_calibration_busy,
                g_full_active != 0u,
                Motion_control_is_reset_safe(),
                printer_reset_quiescent(time_ticks32()),
            };
            const bool duplicate = g_have_last_reset_operation != 0u &&
                                   request.operation_id == g_last_reset_operation_id;
            const AckResult result =
                bmcu_soft_reset::evaluate_request(request, safety, duplicate);
            if (result != ACK_OK)
            {
                send_ack(sequence, kind, result);
                break;
            }
            if (!send_ack(sequence, kind, ACK_OK)) break;

            const uint32_t now = time_ticks32();
            g_reset_operation_id = request.operation_id;
            g_last_reset_operation_id = request.operation_id;
            g_have_last_reset_operation = 1u;
            g_reset_request_reason = request.reason;
            g_reset_deadline_tick = now + static_cast<uint32_t>(request.ttl_ms) * time_hw_tpms;
            g_reset_phase = kResetDrain;
            push_reset_state_event(RESET_SCHEDULED, RESET_CANCEL_NONE);
        }
        break;

    default:
        send_ack(sequence, kind, ACK_UNSUPPORTED);
        break;
    }
}

void reset_rx_parser(uint8_t possible_sync)
{
    g_rx_frame_length = 0u;
    g_rx_expected_length = 0u;
    g_rx_sync_state = possible_sync == kSync0 ? 1u : 0u;
}

void process_one_rx_frame()
{
    while (g_rx_read != g_rx_write)
    {
        const uint8_t value = g_rx[g_rx_read];
        g_rx_read = static_cast<uint8_t>((g_rx_read + 1u) & (kRxRingSize - 1u));

        if (g_rx_sync_state == 0u)
        {
            if (value == kSync0) g_rx_sync_state = 1u;
            continue;
        }
        if (g_rx_sync_state == 1u)
        {
            if (value == kSync1)
            {
                g_rx_sync_state = 2u;
                g_rx_frame_length = 0u;
                g_rx_expected_length = 0u;
            }
            else if (value != kSync0)
            {
                g_rx_sync_state = 0u;
            }
            continue;
        }

        if (g_rx_frame_length >= sizeof(g_rx_frame))
        {
            ++g_rx_frame_error;
            reset_rx_parser(value);
            continue;
        }
        g_rx_frame[g_rx_frame_length++] = value;

        if (g_rx_frame_length == kHeaderSize)
        {
            const uint8_t payload_length = g_rx_frame[4];
            if (payload_length > static_cast<uint8_t>(kMaxDecoded - kHeaderSize - kCrcSize))
            {
                ++g_rx_frame_error;
                reset_rx_parser(value);
                continue;
            }
            g_rx_expected_length = static_cast<uint8_t>(payload_length + kHeaderSize + kCrcSize);
        }

        if (g_rx_expected_length != 0u && g_rx_frame_length == g_rx_expected_length)
        {
            handle_frame(g_rx_frame, g_rx_frame_length);
            reset_rx_parser(0u);
            return;
        }
    }
}

void emit_ams_diag_counter(uint8_t counter, uint32_t value, uint32_t tick)
{
    // No local rate limiter on purpose. The pure module already edge-latches
    // one emission per silence/arming episode, so the bound this path needs is
    // already in place. A tick-difference limiter here would compare a raw
    // 32-bit SysTick delta (18 MHz, wrapping every ~238.6 s) against a 5 s
    // window, so two legitimate emissions separated by ~238.6 s * k would alias
    // into the window and be dropped permanently - the module's edge latch has
    // already been consumed by then, so the event never comes back.
    LogRecord record = {};
    record.header.hw_tick32 = tick;
    record.header.type = RECORD_DIAGNOSTIC_COUNTER;
    record.header.severity = SEVERITY_NOTICE;
    record.header.source = SOURCE_PRINTER_BUS;
    record.header.payload_length = sizeof(LogDiagnosticCounterPayload);
    record.payload.diagnostic_counter.counter = counter;
    record.payload.diagnostic_counter.value = value;
    push_log_record(record);
}

void emit_ams_service_events(const bmcu_ams_service::Events& events, uint32_t tick)
{
    // events.gap_ms is captured inside the module at the instant the edge is
    // raised. Reading the live g_ams_service.gap_now_ms here would report 0 for
    // any edge raised on the frame-arrival path, because note_service() zeroes
    // the gap after poll() latched the event.
    if (events.gap_event)
        emit_ams_diag_counter(DIAG_COUNTER_AMS_SERVICE_GAP_MS, events.gap_ms, tick);
    if (events.reoffer_event)
        emit_ams_diag_counter(DIAG_COUNTER_AMS_WOULD_REOFFER, events.gap_ms, tick);
}
}

void bmcu_link_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};
    DMA_InitTypeDef dma = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    uart.USART_BaudRate = 115200u;
    uart.USART_WordLength = USART_WordLength_9b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_Even;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &uart);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1u;
    nvic.NVIC_IRQChannelSubPriority = 1u;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    dma.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&USART3->DATAR);
    dma.DMA_MemoryBaseAddr = reinterpret_cast<uint32_t>(g_tx[0].data);
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = 1u;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Low;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_DeInit(DMA1_Channel2);
    DMA_Init(DMA1_Channel2, &dma);

    USART_Cmd(USART3, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    g_link_diag = kDiagInit;
    g_last_led_tick = time_ticks32();
    g_status_dirty = BMCU_STATUS_CHANGE_ALL;
    send_hello();
}

void bmcu_link_rx_isr_byte(uint8_t data)
{
    const uint8_t next = static_cast<uint8_t>((g_rx_write + 1u) & (kRxRingSize - 1u));
    if (__builtin_expect(next == g_rx_read, 0))
    {
        ++g_rx_drop;
        return;
    }
    g_rx[g_rx_write] = data;
    g_rx_write = next;
}

void bmcu_link_service(void)
{
    tx_finish_if_complete();
    const uint8_t tx_write_before = g_tx_write;
    process_one_rx_frame();
    bool emitted = g_tx_write != tx_write_before;

    const uint32_t now = time_ticks32();
    tx_fail_if_needed(now);
    tx_recover_if_ready(now);

    if (g_reset_phase != kResetIdle)
    {
        service_soft_reset(now);
        if (g_reset_phase != kResetIdle)
        {
            tx_start_if_idle();
            return;
        }
    }

    const uint32_t one_second = time_hw_tpms * 1000u;
    if (one_second != 0u && static_cast<uint32_t>(now - g_last_led_tick) >= one_second)
    {
        g_last_led_tick = now;
        if (g_led_remaining_s != 0u && --g_led_remaining_s == 0u)
        {
            g_led_mode = 0u;
            g_led_restore_pending = 1u;
            bmcu_link_status_changed(BMCU_STATUS_CHANGE_LED);
        }
    }

    if (!g_status_cache_valid) apply_status_changes(BMCU_STATUS_CHANGE_ALL, false);

    // A full snapshot is a bounded baseline transaction. STATUS can become
    // dirty on every control-loop pass, so allowing it to use each newly freed
    // TX slot can otherwise starve the snapshot forever. Keep EVENT priority,
    // but finish one snapshot record whenever the reserved TX capacity permits;
    // dirty STATUS reasons remain accumulated and are emitted afterwards.
    //
    // Only one frame goes out per cycle, so a sustained event storm would
    // otherwise never let the snapshot advance -- pinning g_full_active and
    // forcing GET_FULL_STATUS to answer ACK_BUSY forever. While a snapshot is
    // draining, alternate event/snapshot each cycle so the snapshot always
    // makes forward progress; when no snapshot is active, EVENT keeps priority.
    if (!emitted && g_full_active)
    {
#if BMCU_LINK_FULL_STATUS
        if (g_full_prefer_record)
        {
            emitted = send_next_full_status_record();
            if (!emitted) emitted = send_next_event();
        }
        else
        {
            emitted = send_next_event();
            if (!emitted) emitted = send_next_full_status_record();
        }
        g_full_prefer_record ^= 1u;
#else
        emitted = send_next_full_status_record();
        if (!emitted) emitted = send_next_event();
#endif
    }
    else if (!emitted)
    {
        emitted = send_next_event();
    }

    if (!emitted && !g_full_active && g_status_dirty != 0u)
    {
        const uint32_t pending = take_status_changes();
        if (pending != 0u)
        {
            apply_status_changes(pending, true);
            if (send_status(g_sequence))
            {
                ++g_sequence;
                emitted = true;
            }
            else
            {
                restore_status_changes(pending);
            }
        }
    }

    tx_start_if_idle();
}

void bmcu_link_apply_led_override(void)
{
    const uint32_t now = time_ticks32();
    if (static_cast<int32_t>(g_rx_diag_until_tick - now) > 0)
    {
        // Cyan: a complete command arrived from the host and passed CRC.
        SYS_RGB.set_RGB(0x00u, 0x30u, 0x30u, 0u);
        return;
    }
    if (g_link_diag == kDiagInit || g_link_diag == kDiagTxPending)
    {
        SYS_RGB.set_RGB(0x00u, 0x00u, 0x30u, 0u);
        return;
    }
    if (g_link_diag == kDiagTxOk)
    {
        if (static_cast<int32_t>(g_diag_until_tick - now) > 0)
        {
            SYS_RGB.set_RGB(0x00u, 0x30u, 0x00u, 0u);
            return;
        }
        g_link_diag = kDiagOff;
    }
    if (g_link_diag == kDiagTxTimeout)
    {
        SYS_RGB.set_RGB(0x30u, 0x00u, 0x30u, 0u);
        return;
    }

    if (g_led_mode == 1u) SYS_RGB.set_RGB(0x00u, 0x00u, 0x30u, 0u);
    else if (g_led_mode == 2u) SYS_RGB.set_RGB(0x00u, 0x30u, 0x00u, 0u);
    else if (g_led_mode == 3u) SYS_RGB.set_RGB(0x30u, 0x00u, 0x30u, 0u);
    else if (g_led_restore_pending)
    {
        if (g_control_error) SYS_RGB.set_RGB(0x10u, 0x00u, 0x00u, 0u);
        else SYS_RGB.set_RGB(0x38u, 0x35u, 0x32u, 0u);
        g_led_restore_pending = 0u;
    }
}

void bmcu_link_set_calibration_busy(bool busy)
{
    g_calibration_busy = busy;
}

void bmcu_link_set_control_error(int error)
{
    if (g_control_error == error) return;
    g_control_error = error;
    if (error) g_printer_bus.online = 0u;
    bmcu_link_status_changed(BMCU_STATUS_CHANGE_ERROR);
}

void bmcu_link_status_changed(uint32_t reasons)
{
    if (reasons == 0u) return;
    // Producer hot path: one OR only. Snapshot/event work stays in service().
    g_status_dirty |= reasons;
}

void bmcu_link_motion_fault(uint8_t channel, uint8_t previous_fault, uint8_t fault)
{
    if (previous_fault == fault) return;
    push_state_event(STATE_FIELD_MOTION_FAULT, channel, previous_fault, fault,
                     SOURCE_SAFETY, fault == 0u ? SEVERITY_NOTICE : SEVERITY_ERROR);
    g_status_dirty |= BMCU_STATUS_CHANGE_MOTION;
}


void bmcu_link_printer_transaction(uint8_t rx_class, uint8_t command, uint8_t outcome,
                                   uint8_t reason, uint16_t request_length,
                                   uint16_t response_length)
{
    const uint32_t tick = time_ticks32();
    if (rx_class == static_cast<uint8_t>(bambubus_package_type::filament_motion_short) ||
        rx_class == static_cast<uint8_t>(bambubus_package_type::filament_motion_long))
    {
        g_last_printer_motion_tick = tick;
        g_have_printer_motion_tick = 1u;
    }
    g_printer_bus.last_rx_class = rx_class;
    g_printer_bus.last_command = command;
    g_printer_bus.last_outcome = outcome;

    const bool rejected = outcome == OUTCOME_REJECTED || outcome == OUTCOME_FAILED;
    uint16_t& rx_counter = rejected ? g_printer_bus.invalid_rx_count : g_printer_bus.valid_rx_count;
    if (rx_counter != 0xFFFFu) ++rx_counter;
    if (!rejected)
    {
        g_printer_bus.online = 1u;
        g_printer_bus.last_valid_tick = tick;
    }
    if (response_length != 0u && g_printer_bus.tx_count != 0xFFFFu) ++g_printer_bus.tx_count;

    if (rejected)
    {
        const bool same_event = g_printer_transaction_event.valid != 0u &&
            g_printer_transaction_event.command == command &&
            g_printer_transaction_event.outcome == outcome &&
            g_printer_transaction_event.reason == reason;
        const uint32_t repeat_interval = time_hw_tpms * 5000u;
        if (same_event && repeat_interval != 0u &&
            static_cast<uint32_t>(tick - g_printer_transaction_event.last_emit_tick) < repeat_interval)
        {
            if (g_printer_transaction_event_suppressed != 0xFFFFFFFFu)
                ++g_printer_transaction_event_suppressed;
            return;
        }
        g_printer_transaction_event.last_emit_tick = tick;
        g_printer_transaction_event.command = command;
        g_printer_transaction_event.outcome = outcome;
        g_printer_transaction_event.reason = reason;
        g_printer_transaction_event.valid = 1u;

        LogRecord record = {};
        record.header.hw_tick32 = tick;
        record.header.type = RECORD_PRINTER_TRANSACTION;
        record.header.severity = SEVERITY_WARNING;
        record.header.source = SOURCE_PRINTER_BUS;
        record.header.payload_length = sizeof(LogPrinterTransactionPayload);
        record.payload.printer_transaction.command = command;
        record.payload.printer_transaction.owner = OWNER_PRINTER;
        record.payload.printer_transaction.outcome = static_cast<TransactionOutcome>(outcome);
        record.payload.printer_transaction.reason = static_cast<DecisionReason>(reason);
        record.payload.printer_transaction.request_length = request_length > 0xFFu
            ? 0xFFu : static_cast<uint8_t>(request_length);
        record.payload.printer_transaction.response_length = response_length > 0xFFu
            ? 0xFFu : static_cast<uint8_t>(response_length);
        push_log_record(record);
    }
}

void bmcu_link_printer_long_transaction(uint16_t type, uint8_t outcome, uint8_t reason,
                                        uint16_t payload_length, uint16_t response_length,
                                        uint8_t payload_hash)
{
    const uint32_t tick = time_ticks32();
    g_printer_auth.last_type = type;
    g_printer_auth.last_payload_length = payload_length;
    g_printer_auth.last_tick = tick;
    g_printer_auth.last_outcome = outcome;
    g_printer_auth.last_reason = reason;
    g_printer_auth.last_response_length = response_length > 0xFFu
        ? 0xFFu : static_cast<uint8_t>(response_length);
    g_printer_auth.last_payload_hash = payload_hash;
    if (type == 0x040Du && g_printer_auth.count_040d != 0xFFFFu)
        ++g_printer_auth.count_040d;
    else if (type == 0x040Eu && g_printer_auth.count_040e != 0xFFFFu)
        ++g_printer_auth.count_040e;

    const bool notable = type == 0x040Du || type == 0x040Eu ||
                         outcome == OUTCOME_REJECTED || outcome == OUTCOME_FAILED;
    if (!notable) return;

    LogRecord record = {};
    record.header.hw_tick32 = tick;
    record.header.type = RECORD_PRINTER_LONG_TRANSACTION;
    record.header.severity = response_length == 0u ? SEVERITY_WARNING : SEVERITY_INFO;
    record.header.source = SOURCE_PRINTER_BUS;
    record.header.payload_length = sizeof(LogPrinterLongTransactionPayload);
    record.payload.printer_long_transaction.type = type;
    record.payload.printer_long_transaction.owner = OWNER_PRINTER;
    record.payload.printer_long_transaction.outcome = static_cast<TransactionOutcome>(outcome);
    record.payload.printer_long_transaction.reason = static_cast<DecisionReason>(reason);
    record.payload.printer_long_transaction.request_length = payload_length > 0xFFu
        ? 0xFFu : static_cast<uint8_t>(payload_length);
    record.payload.printer_long_transaction.response_length = response_length > 0xFFu
        ? 0xFFu : static_cast<uint8_t>(response_length);
    record.payload.printer_long_transaction.payload_hash = payload_hash;
    push_log_record(record);
}

void bmcu_link_ams_service_poll(void)
{
    const uint32_t tick = time_ticks32();
    bmcu_ams_service::Events events = {};
    bmcu_ams_service::poll(g_ams_service, tick, time_hw_tpms, &events);
    emit_ams_service_events(events, tick);
}

void bmcu_link_ams_service_frame(uint8_t service_kind)
{
    if (service_kind >= bmcu_ams_service::kServiceKindCount) return;
    const uint32_t tick = time_ticks32();
    bmcu_ams_service::Events events = {};
    bmcu_ams_service::note_service(g_ams_service, service_kind, tick, time_hw_tpms, &events);
    emit_ams_service_events(events, tick);
}

void bmcu_link_ams_registration_query(void)
{
    const uint32_t tick = time_ticks32();
    bmcu_ams_service::Events events = {};
    bmcu_ams_service::note_query(g_ams_service, tick, time_hw_tpms, &events);
    emit_ams_service_events(events, tick);
}

void bmcu_link_ams_registration_confirm(void)
{
    const uint32_t tick = time_ticks32();
    bmcu_ams_service::Events events = {};
    bmcu_ams_service::note_confirm(g_ams_service, tick, time_hw_tpms, &events);
    emit_ams_service_events(events, tick);
}

void bmcu_link_ams_registration_reset(void)
{
    const uint32_t tick = time_ticks32();
    bmcu_ams_service::Events events = {};
    bmcu_ams_service::note_reset(g_ams_service, tick, time_hw_tpms, &events);
    emit_ams_service_events(events, tick);
}

uint32_t bmcu_link_tx_drop_count(void)
{
    return g_tx_drop + g_event_drop;
}

bool bmcu_link_reset_pending(void)
{
    return g_reset_phase != kResetIdle;
}

#endif /* BMCU_LINK_ENABLED */
