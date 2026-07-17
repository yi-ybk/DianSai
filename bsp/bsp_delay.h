/**
 * @file    bsp_delay.h
 * @brief   不依赖操作系统调度器的阻塞延时接口
 */
#pragma once

#include <stdint.h>

/**
 * @brief 阻塞延时指定的微秒数
 * @param us 延时时间，单位为微秒；传入 0 时立即返回
 * @note  延时期间 CPU 持续忙等待，不会主动让出处理器。
 */
void BSP_DelayUs(uint32_t us);

/**
 * @brief 阻塞延时指定的毫秒数
 * @param ms 延时时间，单位为毫秒；传入 0 时立即返回
 * @note  适合调度器启动前或短时序延时，任务中的长延时应使用 osDelay()。
 */
void BSP_DelayMs(uint32_t ms);
