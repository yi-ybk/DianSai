/**
 * @file    tasks.h
 * @brief   系统任务相关头文件
 * @details 负责声明FreeRTOS任务相关的对象和创建函数。
 */
#pragma once

#include "cmsis_os.h"
#include "imu_driver.h"

extern Imu_t imu0;

void ImuParseTask(void *argument);
