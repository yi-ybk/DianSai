/**
 * @file    led_driver.h
 * @brief   LED灯模块驱动头文件
 * @details 基于bsp_gpio封装LED初始化、亮灭、翻转和状态读取接口，支持高电平点亮和低电平点亮。
 */
#pragma once

#include "bsp_gpio.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief LED逻辑状态 */
typedef enum
{
    LED_STATE_OFF = 0, /**< LED熄灭 */
    LED_STATE_ON,      /**< LED点亮 */
} LedState_t;

/** @brief LED初始化配置 */
typedef struct
{
    GPIO_TypeDef *GPIOx;          /**< LED GPIO端口 */
    uint32_t GPIO_Pin;            /**< LED GPIO引脚 */
    GPIO_PinState active_state;   /**< LED点亮时的GPIO电平 */
    LedState_t init_state;        /**< 初始化后的LED状态 */
    void *id;                     /**< 用户自定义标识 */
} LedInitConfig_t;

/** @brief LED运行数据 */
typedef struct
{
    LedState_t state;             /**< 当前逻辑状态 */
    uint32_t toggle_count;        /**< 翻转次数 */
} LedData_t;

typedef struct Led Led_t;

/** @brief LED对象 */
struct Led
{
    GPIOInstance *gpio;           /**< 底层GPIO对象 */
    bool initialized;             /**< 初始化标记 */
    LedInitConfig_t init_config;  /**< 初始化配置缓存 */
    LedData_t data;               /**< 当前运行数据 */

    bool (*init)(Led_t *led, const LedInitConfig_t *config);
    void (*on)(Led_t *led);
    void (*off)(Led_t *led);
    void (*toggle)(Led_t *led);
    void (*write)(Led_t *led, LedState_t state);
    LedState_t (*read)(Led_t *led);
    void (*get_data)(Led_t *led, LedData_t *data);
};

/** @brief 初始化LED对象 */
bool LedInit(Led_t *led, const LedInitConfig_t *config);
/** @brief 点亮LED */
void LedOn(Led_t *led);
/** @brief 熄灭LED */
void LedOff(Led_t *led);
/** @brief 翻转LED状态 */
void LedToggle(Led_t *led);
/** @brief 写入LED逻辑状态 */
void LedWrite(Led_t *led, LedState_t state);
/** @brief 读取LED逻辑状态 */
LedState_t LedRead(Led_t *led);
/** @brief 获取LED运行数据 */
void LedGetData(Led_t *led, LedData_t *data);

#define LED_OBJECT_DEFAULT        \
        .init     = LedInit,      \
        .on       = LedOn,        \
        .off      = LedOff,       \
        .toggle   = LedToggle,    \
        .write    = LedWrite,     \
        .read     = LedRead,      \
        .get_data = LedGetData
