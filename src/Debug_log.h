/**
 * @file Debug_log.h
 * @brief 调试日志输出头文件
 *
 * 通过 USART3（PB10/PB11）输出调试信息。
 * 默认关闭（Debug_log_on 被注释），开启后会占用 PB10/PB11 引脚。
 * 注意：开启调试日志后，PB10/PB11 不可用于 I2C2。
 *
 * 使用方法：
 * 1. 取消注释 #define Debug_log_on 以启用调试输出
 * 2. 调用 DEBUG("message") 输出字符串
 * 3. 调用 DEBUG_num("prefix", value) 输出带数值的信息
 */

#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdint.h>
#include "hal/time_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 调试日志开关（取消注释以启用） */
//#define Debug_log_on
/* 调试串口波特率 */
#define Debug_log_baudrate 115200

/** 延时初始化（空操作，兼容 Arduino 风格） */
static inline void Delay_Init(void) { }
/** 微秒级延时 */
static inline void Delay_Us(uint32_t n) { delay_us(n); }
/** 毫秒级延时 */
static inline void Delay_Ms(uint32_t n) { delay(n); }

/** 初始化调试串口 */
void Debug_log_init(void);
/** 获取 64 位时间戳（当前返回 0） */
uint64_t Debug_log_count64(void);
/** 输出时间信息（当前为空操作） */
void Debug_log_time(void);
/** 输出字符串（自动计算长度） */
void Debug_log_write(const void *data);
/** 输出指定长度的数据 */
void Debug_log_write_num(const void *data, int num);

/** 时间日志宏（当前为空操作） */
#define DEBUG_time_log()

/* 调试宏定义：根据 Debug_log_on 开关选择实际输出或空操作 */
#ifdef Debug_log_on
  #define DEBUG_init()        Debug_log_init()
  #define DEBUG(logs)         Debug_log_write(logs)
  #define DEBUG_num(logs,num) Debug_log_write_num((logs),(num))
  #define DEBUG_time()        Debug_log_time()
  #define DEBUG_get_time()    Debug_log_count64()
#else
  /* 调试关闭时，所有宏展开为空操作，不产生任何代码 */
  #define DEBUG_init()        do{}while(0)
  #define DEBUG(logs)         do{}while(0)
  #define DEBUG_num(logs,num) do{}while(0)
  #define DEBUG_time()        do{}while(0)
  #define DEBUG_get_time()    (0ULL)
#endif

#ifdef __cplusplus
}
#endif

#endif
