/**
 * @file    electromagnet.h
 * @brief   电磁铁MOS驱动模块控制接口
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化控制引脚并默认关闭电磁铁 */
void ElectromagnetInit(void);
/** @brief 打开电磁铁 */
void ElectromagnetOn(void);
/** @brief 关闭电磁铁 */
void ElectromagnetOff(void);
/** @brief 设置电磁铁开关状态 */
void ElectromagnetSet(bool enabled);
/** @brief 翻转电磁铁开关状态 */
void ElectromagnetToggle(void);
/** @brief 查询当前控制输出状态 */
bool ElectromagnetIsOn(void);

#ifdef __cplusplus
}
#endif
