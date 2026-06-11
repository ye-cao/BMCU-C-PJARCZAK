/**
 * @file main.cpp
 * @brief BMCU-C 固件主程序入口
 *
 * 本文件是 BMCU-C 智能料架控制器的主程序。
 * 功能概览：
 * - 初始化系统时钟、GPIO、定时器
 * - 初始化 RGB LED 状态指示灯
 * - 初始化 SHT45 温湿度传感器（通过 I2C2）
 * - 初始化 AMS（自动供料系统）数据结构
 * - 初始化 Flash 存储（保存/恢复耗材参数和加载状态）
 * - 初始化 ADC/DMA（模拟量采集）
 * - 初始化运动控制（PID + AS5600 编码器 + 电机 PWM）
 * - 初始化 BambuBus/Hub 总线通信
 * - 主循环：处理总线协议、运动控制、LED 更新、温湿度采集
 */

#include "MC_PULL_calibration.h"
#include "ws2812.h"

#include "Flash_saves.h"
#include "Motion_control.h"
#include "_bus_hardware.h"
#include "ams.h"
#include "ahub_bus.h"
#include "bambu_bus_ams.h"
#include "ADC_DMA.h"
#include "Debug_log.h"
#include "sht45.h"
#include <string.h>

/* 系统 RGB LED 实例（1 颗，PD1 引脚） */
WS2812_class SYS_RGB;
/* 4 个通道的输出 RGB LED 实例（每个通道 2 颗 LED） */
WS2812_class RGBOUT[4];

/**
 * @brief RGB LED 硬件初始化
 *
 * 将所有 RGB LED 控制器绑定到对应的 GPIO 引脚：
 * - SYS_RGB：PD1（系统状态指示灯，1 颗）
 * - RGBOUT[0]：PA11（通道 A，2 颗）
 * - RGBOUT[1]：PA8 （通道 B，2 颗）
 * - RGBOUT[2]：PB1 （通道 C，2 颗）
 * - RGBOUT[3]：PB0 （通道 D，2 颗）
 */
void RGB_init()
{
    SYS_RGB.init(1, GPIOD, GPIO_Pin_1);
    RGBOUT[0].init(2, GPIOA, GPIO_Pin_11);
    RGBOUT[1].init(2, GPIOA, GPIO_Pin_8);
    RGBOUT[2].init(2, GPIOB, GPIO_Pin_1);
    RGBOUT[3].init(2, GPIOB, GPIO_Pin_0);
}

/**
 * @brief RGB LED 更新显示
 *
 * 仅当有 LED 数据变更时才执行更新。
 * 使用 1ms 最小间隔限制刷新率，避免过频刷新影响 CPU 性能。
 * 调用各 LED 控制器的 updata() 方法发送 WS2812 数据。
 */
void RGB_update()
{
    /* 检查是否有任何 LED 需要更新 */
    if (!(SYS_RGB.is_dirty() ||
          RGBOUT[0].is_dirty() || RGBOUT[1].is_dirty() ||
          RGBOUT[2].is_dirty() || RGBOUT[3].is_dirty()))
        return;

    static uint32_t last = 0u;

    /* 最小刷新间隔为 1ms */
    uint32_t min_gap = time_hw_tpms;
    if (!min_gap) min_gap = 1u;

    const uint32_t now = time_ticks32();
    if (last != 0u && (uint32_t)(now - last) < min_gap)
        return;

    last = now;

    /* 逐个更新 LED 控制器 */
    SYS_RGB.updata();
    RGBOUT[0].updata();
    RGBOUT[1].updata();
    RGBOUT[2].updata();
    RGBOUT[3].updata();
}

/* ========== 耗材数据 Flash 存储管理 ========== */

/**
 * 耗材数据脏位掩码，bit0~3 分别对应 4 个耗材槽位。
 * 置位表示对应槽位的数据已修改，需要写入 Flash。
 */
static uint8_t g_fil_dirty = 0;

/**
 * 当前已加载（送入打印机）的耗材通道号。
 * 0xFF 表示无耗材加载。
 */
static uint8_t g_loaded_ch = 0xFF;

/**
 * 加载状态脏标志，置位表示需要将加载状态写入 Flash。
 */
static uint8_t g_state_dirty = 0;

/**
 * @brief 将 RAM 中的耗材信息转换为 Flash 存储格式
 *
 * @param fil  耗材槽位索引（0~3）
 * @param o    输出的 Flash_FilamentInfo 结构体指针
 *
 * 从 ams[BAMBU_BUS_AMS_NUM].filament[fil] 读取耗材信息，
 * 拷贝到 Flash 可存储的紧凑格式中。
 */
static inline void ram_to_flashinfo(uint8_t fil, Flash_FilamentInfo* o)
{
    const _filament* f = &ams[BAMBU_BUS_AMS_NUM].filament[fil];

    memcpy(o->bambubus_filament_id, f->bambubus_filament_id, sizeof(o->bambubus_filament_id));
    o->color_R = f->color_R;
    o->color_G = f->color_G;
    o->color_B = f->color_B;
    o->color_A = f->color_A;
    o->temperature_min = f->temperature_min;
    o->temperature_max = f->temperature_max;
    memcpy(o->name, f->name, sizeof(o->name));
}

/**
 * @brief 将 Flash 存储格式的耗材信息加载到 RAM
 *
 * @param fil  耗材槽位索引（0~3）
 * @param i    输入的 Flash_FilamentInfo 结构体指针
 *
 * 从 Flash 读取的数据拷贝到 ams[BAMBU_BUS_AMS_NUM].filament[fil]，
 * 名称字段会清零并确保末尾有 '\0' 终止符。
 */
static inline void flashinfo_to_ram(uint8_t fil, const Flash_FilamentInfo* i)
{
    _filament* f = &ams[BAMBU_BUS_AMS_NUM].filament[fil];

    memcpy(f->bambubus_filament_id, i->bambubus_filament_id, sizeof(i->bambubus_filament_id));
    f->color_R = i->color_R;
    f->color_G = i->color_G;
    f->color_B = i->color_B;
    f->color_A = i->color_A;
    f->temperature_min = i->temperature_min;
    f->temperature_max = i->temperature_max;

    /* 清零后拷贝名称，防止残留数据 */
    memset(f->name, 0, sizeof(f->name));
    memcpy(f->name, i->name, sizeof(i->name));
    f->name[sizeof(f->name) - 1u] = 0;
}

/**
 * @brief 从 Flash 读取所有 4 个耗材槽位的数据
 *
 * @return true=至少读取了一个槽位的数据
 */
bool ams_datas_read()
{
    bool any = false;

    for (uint8_t fil = 0; fil < 4u; fil++)
    {
        Flash_FilamentInfo fi;
        if (Flash_AMS_filament_read(fil, &fi))
        {
            flashinfo_to_ram(fil, &fi);
            any = true;
        }
    }

    return any;
}

/**
 * @brief 标记所有 4 个耗材槽位需要保存
 */
void ams_datas_set_need_to_save()
{
    g_fil_dirty = 0x0Fu;
}

/**
 * @brief 标记指定耗材槽位需要保存
 *
 * @param filament_idx  耗材槽位索引（0~3）
 */
void ams_datas_set_need_to_save_filament(uint8_t filament_idx)
{
    if (filament_idx >= 4u) return;
    g_fil_dirty |= (uint8_t)(1u << filament_idx);
}

/**
 * @brief 标记耗材已加载（送入打印头）
 *
 * @param filament_ch  耗材通道号（0~3）
 *
 * 仅在当前无耗材加载时有效（g_loaded_ch == 0xFF），
 * 防止重复加载覆盖已有状态。
 */
void ams_state_set_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu) return;
    g_loaded_ch = filament_ch;
    g_state_dirty = 1u;
}

/**
 * @brief 标记耗材已卸载（从打印头退出）
 *
 * @param filament_ch  耗材通道号（0~3）
 *
 * 只有当前加载的通道与请求卸载的通道一致时才执行。
 * 通道号为 0xFF 时表示无条件卸载。
 */
void ams_state_set_unloaded(uint8_t filament_ch)
{
    if (g_loaded_ch == 0xFFu) return;
    if (filament_ch < 4u && g_loaded_ch != filament_ch) return;
    g_loaded_ch = 0xFFu;
    g_state_dirty = 1u;
}

/**
 * @brief 获取当前已加载的耗材通道号
 *
 * @return 通道号（0~3），0xFF 表示无耗材加载
 */
uint8_t ams_state_get_loaded(void)
{
    return g_loaded_ch;
}

/**
 * @brief 执行耗材加载状态的 Flash 保存
 *
 * 仅在状态脏标志置位时执行写入。
 * 写入成功后清除脏标志。
 */
static void ams_state_save_run()
{
    if (!g_state_dirty) return;

    if (Flash_AMS_state_write(g_loaded_ch))
        g_state_dirty = 0u;
}

/**
 * @brief 执行耗材数据的 Flash 保存（分时写入）
 *
 * 每次主循环调用时仅保存一个脏槽位，避免一次性写入耗时过长。
 * 按 bit0~bit3 顺序查找第一个脏槽位并写入。
 */
void ams_datas_save_run()
{
    if (!g_fil_dirty) return;

    /* 查找第一个需要保存的槽位 */
    uint8_t fil = 0xFFu;
    for (uint8_t i = 0; i < 4u; i++)
    {
        if (g_fil_dirty & (uint8_t)(1u << i))
        {
            fil = i;
            break;
        }
    }

    if (fil == 0xFFu) return;

    /* 转换并写入 Flash */
    Flash_FilamentInfo now;
    ram_to_flashinfo(fil, &now);

    if (Flash_AMS_filament_write(fil, &now))
        g_fil_dirty &= (uint8_t)~(1u << fil);
}

/**
 * @brief SHT45 温湿度更新到所有耗材槽位
 *
 * 读取一次 SHT45 的温度和湿度，然后将结果写入当前 AMS
 * 所有 4 个耗材槽位的 compartment_temperature 和 compartment_humidity 字段。
 * 这些数据会通过 BambuBus 协议上报给打印机。
 */
static void sht45_update_filament(void)
{
    float temp = 0.0f;
    float humi = 0.0f;
    uint8_t ok_t = sht45_read_temperature(&temp);
    uint8_t ok_h = sht45_read_humidity(&humi);

    _ams *a = &ams[BAMBU_BUS_AMS_NUM];
    for (uint8_t i = 0; i < 4u; i++)
    {
        if (!ok_t)
            a->filament[i].compartment_temperature = (int8_t)(temp + 0.5f);
        if (!ok_h)
            a->filament[i].compartment_humidity = (uint8_t)(humi + 0.5f);
    }
}

/* ========== 主函数 ========== */

/**
 * @brief 程序入口
 *
 * 初始化顺序：
 * 1. 系统时钟和硬件抽象层
 * 2. 中断控制器和看门狗
 * 3. GPIO 重映射（PD0/PD1）
 * 4. RGB LED
 * 5. 调试串口（USART3，当 Debug_log_on 启用时）
 * 6. SHT45 温湿度传感器
 * 7. AMS 数据结构
 * 8. Flash 存储
 * 9. ADC/DMA 模拟量采集
 * 10. 拉力校准
 * 11. 从 Flash 恢复上次的耗材数据和加载状态
 * 12. 运动控制（PID + 编码器 + 电机）
 * 13. BambuBus 总线初始化
 *
 * 主循环：
 * 1. 运行 AHUB/BambuBus 协议处理
 * 2. 处理错误状态和 LED 指示
 * 3. 周期性保存耗材数据和加载状态
 * 4. 执行运动控制（PID 调节、送料/退料）
 * 5. 更新 RGB LED 显示
 * 6. 每 2 秒读取一次 SHT45 温湿度数据
 */
int main(void)
{
    /* 系统初始化：配置时钟树、Flash 预取等 */
    SystemInit();
    SystemCoreClockUpdate();
    /* 硬件定时器初始化：提供精确的 ms/us 级延时 */
    time_hw_init();

    /* 使能全局中断 */
    __enable_irq();

    /* 禁用看门狗（WWDG）：该固件不需要看门狗复位 */
    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    /* 使能 AFIO 时钟（用于 GPIO 重映射） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* 配置 NVIC 优先级分组：Group1 = 1位抢占 + 3位子优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /* PD0/PD1 重映射为普通 GPIO（用于 AS5600 编码器 SDA 通道 0） */
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

    /* 初始化 RGB LED 硬件 */
    RGB_init();
    delay(10);

    /* 开机指示：系统 LED 显示红色，所有通道 LED 关闭 */
    SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
    for (int i = 0; i < 4; i++) RGBOUT[i].set_RGB(0, 0, 0, 0);
    RGB_update();
    delay(50);

    /* 初始化各子系统 */
    DEBUG_init();          /* 调试串口（可选） */
    sht45_init();          /* SHT45 温湿度传感器 */
    ams_init();            /* AMS 数据结构初始化 */
    Flash_saves_init();    /* Flash 存储初始化 */

    /* ADC/DMA 初始化并等待首次采样完成 */
    ADC_DMA_init();
    ADC_DMA_wait_full();

    /* 上电拉力校准 */
    MC_PULL_calibration_boot();
    /* 从 Flash 恢复耗材参数 */
    ams_datas_read();

    /* 恢复上次的耗材加载状态 */
    {
        uint8_t ch = 0xFFu;
        if (Flash_AMS_state_read(&ch))
        {
            g_loaded_ch = ch;

            /* 如果有耗材加载记录，恢复对应槽位状态 */
            if (ch < 4u)
            {
                _ams* a = &ams[BAMBU_BUS_AMS_NUM];

                a->now_filament_num  = ch;         /* 当前耗材编号 */
                a->filament_use_flag = 0x04;        /* 耗材使用标志 */
                a->pressure          = 0x2B00;      /* 送料压力 */

                /* 所有通道设为空闲，仅当前加载通道设为"使用中" */
                for (uint8_t i = 0; i < 4u; i++)
                    a->filament[i].motion = _filament_motion::idle;

                a->filament[ch].motion = _filament_motion::on_use;
            }
        }
    }

    /* 初始化运动控制（PID + AS5600 编码器 + 电机 PWM） */
    Motion_control_init();
    /* 初始化 BambuBus 总线通信 */
    bambubus_init();
    /* 初始化底层串口通信 */
    bus_init();

    DEBUG("START\n");

    /* ========== 主循环 ========== */
    while (1)
    {
        /* 运行 AHUB 总线协议处理 */
        const ahubus_package_type   ahub_stu     = ahubus_run();
        /* 运行 BambuBus 总线协议处理 */
        const bambubus_package_type bambubus_stu = bambubus_run();
        /* 发送待发送的总线数据包 */
        bus_port_to_host.send_package();

        static int error = 0;

        /* 处理总线协议结果 */
        if ((ahub_stu != ahubus_package_type::none) || (bambubus_stu != bambubus_package_type::none))
        {
            if ((ahub_stu != ahubus_package_type::error) || (bambubus_stu != bambubus_package_type::error))
            {
                error = 0;

                /* 收到 BambuBus 心跳：设置系统 LED 为青色，标记为 AMS 设备 */
                if (bambubus_stu == bambubus_package_type::heartbeat)
                {
                    SYS_RGB.set_RGB(0x38, 0x35, 0x32, 0);
                    bus_host_device_type = host_device_type_ams;
                }

                /* 收到 AHUB 心跳：标记为 AHUB 设备 */
                if (ahub_stu == ahubus_package_type::heartbeat)
                    bus_host_device_type = host_device_type_ahub;

                /* 周期性保存耗材数据和加载状态到 Flash */
                ams_datas_save_run();
                ams_state_save_run();
            }
            else
            {
                /* 通信错误：系统 LED 显示红色 */
                error = -1;
                SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
            }
        }

        /* 运动控制：PID 调节、送料/退料动作、LED 状态更新 */
        Motion_control_run(error);
        /* 更新 RGB LED 显示 */
        RGB_update();

        /* 每 2 秒读取一次 SHT45 温湿度数据并更新到耗材结构体 */
        static uint32_t last_sht45_ms = 0;
        uint32_t now_ms = (uint32_t)(time_ms64() & 0xFFFFFFFF);
        if ((now_ms - last_sht45_ms) >= 2000u)
        {
            last_sht45_ms = now_ms;
            sht45_update_filament();
        }
    }
}
