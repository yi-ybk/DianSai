/**
 * @file    bsp_tim.h
 * @brief   定时器底层驱动接口声明
 * @details 提供基础定时器、编码器模式定时器的注册、启停和计数相关接口。
 */
#pragma once

#include "mspm0_hal_compat.h"
#include <stdint.h>

#define TIM_DEVICE_CNT 7U

typedef struct TIMInstance TIMInstance;

/**
 * @brief 定时器模块回调函数类型
 * @param instance 触发回调的定时器实例
 */
typedef void (*tim_module_callback)(TIMInstance *instance);

/** @brief 定时器工作模式 */
typedef enum
{
    TIM_MODE_BASE = 0,     /**< 基础定时器 */
    TIM_MODE_ENCODER,      /**< 编码器定时器 */
} TIMMode_t;

/** @brief 定时器启动方式 */
typedef enum
{
    TIM_START_NONE = 0,    /**< 不自动启动 */
    TIM_START_POLLING,     /**< 轮询方式启动 */
    TIM_START_IT,          /**< 中断方式启动 */
} TIMStartMode_t;

/** @brief 定时器实例对象 */
struct TIMInstance
{
    TIM_HandleTypeDef *htim;             /**< HAL定时器句柄 */
    TIMMode_t mode;                      /**< 定时器工作模式 */
    TIMStartMode_t start_mode;           /**< 启动方式 */
    uint32_t channel;                    /**< 使用的通道 */
    uint32_t last_counter;               /**< 上一次记录的计数值 */
    tim_module_callback module_callback; /**< 模块回调函数 */
    void *id;                            /**< 用户自定义上下文 */
};

/** @brief 定时器初始化配置 */
typedef struct
{
    TIM_HandleTypeDef *htim;             /**< HAL定时器句柄 */
    TIMMode_t mode;                      /**< 工作模式 */
    TIMStartMode_t start_mode;           /**< 启动方式 */
    uint32_t channel;                    /**< 通道 */
    tim_module_callback module_callback; /**< 模块回调函数 */
    void *id;                            /**< 用户上下文 */
} TIM_Init_Config_s;

/** @brief 注册一个定时器实例 */
TIMInstance *TIMRegister(TIM_Init_Config_s *config);
/** @brief 启动定时器实例 */
uint8_t TIMStart(TIMInstance *instance);
/** @brief 停止定时器实例 */
uint8_t TIMStop(TIMInstance *instance);
/** @brief 获取当前计数值 */
uint32_t TIMGetCounter(TIMInstance *instance);
/** @brief 设置当前计数值 */
void TIMSetCounter(TIMInstance *instance, uint32_t counter);
/** @brief 获取自动重装载值 */
uint32_t TIMGetAutoReload(TIMInstance *instance);
/** @brief 设置自动重装载值 */
void TIMSetAutoReload(TIMInstance *instance, uint32_t autoreload);
/** @brief 判断是否处于向下计数 */
uint8_t TIMIsCountingDown(TIMInstance *instance);
/** @brief 根据当前计数与上次计数计算增量 */
int32_t TIMGetDelta(TIMInstance *instance, uint32_t current_counter);
/** @brief HAL定时器周期到达回调入口 */
void TIMPeriodElapsedCallback(TIM_HandleTypeDef *htim);
