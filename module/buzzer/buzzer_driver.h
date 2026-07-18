/**
 * @file    buzzer_driver.h
 * @brief   蜂鸣器模块驱动头文件
 * @details 基于bsp_gpio和bsp_pwm封装有源、无源蜂鸣器控制，并提供对象式访问接口。
 */
#pragma once

#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 蜂鸣器类型 */
typedef enum
{
    BUZZER_TYPE_ACTIVE_GPIO = 0, /**< 有源蜂鸣器，使用GPIO控制 */
    BUZZER_TYPE_PASSIVE_PWM,     /**< 无源蜂鸣器，使用PWM控制 */
} BuzzerType_t;

/** @brief 蜂鸣器逻辑状态 */
typedef enum
{
    BUZZER_STATE_OFF = 0, /**< 停止鸣响 */
    BUZZER_STATE_ON,      /**< 开始鸣响 */
} BuzzerState_t;

/** @brief 有源蜂鸣器GPIO配置 */
typedef struct
{
    GPIO_TypeDef *GPIOx;        /**< GPIO端口 */
    uint32_t GPIO_Pin;          /**< GPIO引脚 */
    GPIO_PinState active_state; /**< 蜂鸣器鸣响时的GPIO电平 */
} BuzzerGpioConfig_t;

/** @brief 无源蜂鸣器PWM配置 */
typedef struct
{
    TIM_HandleTypeDef *htim; /**< PWM定时器句柄 */
    uint32_t channel;        /**< PWM通道 */
    float frequency_hz;      /**< 初始鸣响频率(Hz) */
    float duty_ratio;        /**< 初始占空比，取值范围0.0~1.0 */
} BuzzerPwmConfig_t;

/** @brief 蜂鸣器初始化配置 */
typedef struct
{
    BuzzerType_t type;           /**< 蜂鸣器类型 */
    BuzzerGpioConfig_t gpio;     /**< 有源蜂鸣器GPIO配置 */
    BuzzerPwmConfig_t pwm;       /**< 无源蜂鸣器PWM配置 */
    BuzzerState_t init_state;    /**< 初始化后的逻辑状态 */
    void *id;                    /**< 用户自定义标识 */
} BuzzerInitConfig_t;

/** @brief 蜂鸣器运行数据 */
typedef struct
{
    BuzzerType_t type;       /**< 当前蜂鸣器类型 */
    BuzzerState_t state;     /**< 当前逻辑状态 */
    float frequency_hz;      /**< 当前无源蜂鸣器频率(Hz) */
    float duty_ratio;        /**< 当前无源蜂鸣器占空比 */
    uint32_t on_count;       /**< 从停止切换到鸣响的次数 */
} BuzzerData_t;

typedef struct Buzzer Buzzer_t;

/** @brief 蜂鸣器对象 */
struct Buzzer
{
    GPIOInstance *gpio;             /**< 有源蜂鸣器底层GPIO对象 */
    PWMInstance *pwm;               /**< 无源蜂鸣器底层PWM对象 */
    bool initialized;               /**< 初始化标识 */
    BuzzerInitConfig_t init_config; /**< 初始化配置缓存 */
    BuzzerData_t data;              /**< 当前运行数据 */

    bool (*init)(Buzzer_t *buzzer, const BuzzerInitConfig_t *config);
    void (*on)(Buzzer_t *buzzer);
    void (*off)(Buzzer_t *buzzer);
    void (*toggle)(Buzzer_t *buzzer);
    void (*write)(Buzzer_t *buzzer, BuzzerState_t state);
    bool (*set_tone)(Buzzer_t *buzzer, float frequency_hz, float duty_ratio);
    void (*get_data)(Buzzer_t *buzzer, BuzzerData_t *data);
};

/** @brief 初始化蜂鸣器对象 */
bool BuzzerInit(Buzzer_t *buzzer, const BuzzerInitConfig_t *config);
/** @brief 启动蜂鸣器 */
void BuzzerOn(Buzzer_t *buzzer);
/** @brief 停止蜂鸣器 */
void BuzzerOff(Buzzer_t *buzzer);
/** @brief 翻转蜂鸣器状态 */
void BuzzerToggle(Buzzer_t *buzzer);
/** @brief 写入蜂鸣器逻辑状态 */
void BuzzerWrite(Buzzer_t *buzzer, BuzzerState_t state);
/** @brief 设置无源蜂鸣器频率和占空比 */
bool BuzzerSetTone(Buzzer_t *buzzer, float frequency_hz, float duty_ratio);
/** @brief 获取蜂鸣器运行数据 */
void BuzzerGetData(Buzzer_t *buzzer, BuzzerData_t *data);

#define BUZZER_OBJECT_DEFAULT       \
        .init     = BuzzerInit,     \
        .on       = BuzzerOn,       \
        .off      = BuzzerOff,      \
        .toggle   = BuzzerToggle,   \
        .write    = BuzzerWrite,    \
        .set_tone = BuzzerSetTone,  \
        .get_data = BuzzerGetData
