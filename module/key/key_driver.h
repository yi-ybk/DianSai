/**
 * @file    key_driver.h
 * @brief   按键模块驱动头文件
 * @details 基于bsp_gpio外部中断回调封装按键初始化、状态读取和事件回调接口。
 */
#pragma once

#include "bsp_gpio.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 按键逻辑状态 */
typedef enum
{
    KEY_STATE_RELEASED = 0, /**< 按键释放 */
    KEY_STATE_PRESSED,      /**< 按键按下 */
} KeyState_t;

/** @brief 按键事件类型 */
typedef enum
{
    KEY_EVENT_NONE = 0, /**< 无事件 */
    KEY_EVENT_PRESS,    /**< 按下事件 */
    KEY_EVENT_RELEASE,  /**< 释放事件 */
} KeyEvent_t;

typedef struct Key Key_t;

/**
 * @brief 按键事件回调函数
 * @note  该回调在GPIO外部中断上下文中执行，应保持短小，避免阻塞调用。
 */
typedef void (*KeyEventCallback_t)(Key_t *key, KeyEvent_t event, void *context);

/** @brief 按键初始化配置 */
typedef struct
{
    GPIO_TypeDef *GPIOx;                /**< 按键GPIO端口 */
    uint32_t GPIO_Pin;                  /**< 按键GPIO引脚 */
    GPIO_PinState active_state;         /**< 按键按下时的GPIO电平 */
    GPIO_EXTI_MODE_e exti_mode;         /**< 外部中断触发模式，应与CubeMX配置一致 */
    uint32_t debounce_ms;               /**< 中断消抖时间(ms)，为0时关闭消抖 */
    KeyEventCallback_t event_callback;  /**< 按键事件回调函数 */
    void *event_context;                /**< 传给事件回调的用户上下文 */
} KeyInitConfig_t;

/** @brief 按键运行数据 */
typedef struct
{
    KeyState_t state;             /**< 当前按键逻辑状态 */
    KeyEvent_t last_event;        /**< 最近一次按键事件 */
    uint32_t event_count;         /**< 事件总次数 */
    uint32_t press_count;         /**< 按下事件次数 */
    uint32_t release_count;       /**< 释放事件次数 */
    uint32_t last_event_tick;     /**< 最近一次有效事件时间戳 */
} KeyData_t;

/** @brief 按键对象 */
struct Key
{
    GPIOInstance *gpio;           /**< 底层GPIO对象 */
    bool initialized;             /**< 初始化标记 */
    KeyInitConfig_t init_config;  /**< 初始化配置缓存 */
    KeyData_t data;               /**< 当前运行数据 */

    bool (*init)(Key_t *key, const KeyInitConfig_t *config);
    KeyState_t (*read)(Key_t *key);
    KeyEvent_t (*get_last_event)(Key_t *key);
    void (*clear_event)(Key_t *key);
    void (*set_callback)(Key_t *key, KeyEventCallback_t callback, void *context);
    void (*get_data)(Key_t *key, KeyData_t *data);
};

/** @brief 初始化按键对象 */
bool KeyInit(Key_t *key, const KeyInitConfig_t *config);
/** @brief 读取按键逻辑状态 */
KeyState_t KeyRead(Key_t *key);
/** @brief 获取最近一次按键事件 */
KeyEvent_t KeyGetLastEvent(Key_t *key);
/** @brief 清除最近一次按键事件 */
void KeyClearEvent(Key_t *key);
/** @brief 设置按键事件回调函数 */
void KeySetCallback(Key_t *key, KeyEventCallback_t callback, void *context);
/** @brief 获取按键运行数据 */
void KeyGetData(Key_t *key, KeyData_t *data);

#define KEY_OBJECT_DEFAULT            \
        .init           = KeyInit,    \
        .read           = KeyRead,    \
        .get_last_event = KeyGetLastEvent, \
        .clear_event    = KeyClearEvent,   \
        .set_callback   = KeySetCallback,  \
        .get_data       = KeyGetData
