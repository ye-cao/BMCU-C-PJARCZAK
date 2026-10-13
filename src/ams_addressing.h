#pragma once
#include <stdint.h>

#ifndef BAMBU_BUS_AMS_NUM
#define BAMBU_BUS_AMS_NUM 0
#endif

// 0 - 3 = fixed AMS A - D
// 4 = AUTO
#if (BAMBU_BUS_AMS_NUM < 0) || (BAMBU_BUS_AMS_NUM > 4)
#error "BAMBU_BUS_AMS_NUM must be 0..3, or 4 for AUTO"
#endif

/* ---------------------------------------------------------------------------
 * ⚠ 重要：AUTO 寻址下 BAMBU_BUS_AMS_NUM == 4，而 ams[] 数组长度是 ams_max_number
 *   (= 4)，所以 `ams[BAMBU_BUS_AMS_NUM]` 会越界（读到的其实是 ams[] 之后其它
 *   全局变量，filament[].online 会恒为 false）。
 *
 *   访问“本机自己是哪个 AMS 槽位”的数据，只能二选一：
 *       · BMCU_LOCAL_AMS_INDEX          编译期常量，零开销（本文件定义）
 *       · bambubus_local_ams_index()    上游惯用法，同样返回 BMCU_LOCAL_AMS_INDEX
 *   （上游源码里从不出现 ams[BAMBU_BUS_AMS_NUM]，全部走上面两者之一）
 * ------------------------------------------------------------------------- */
#if BAMBU_BUS_AMS_NUM == 4
#define BMCU_AMS_AUTO 1
#define BMCU_LOCAL_AMS_INDEX 0
#else
#define BMCU_AMS_AUTO 0
#define BMCU_LOCAL_AMS_INDEX BAMBU_BUS_AMS_NUM
#endif
