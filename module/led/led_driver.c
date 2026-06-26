/**
 * @file    led_driver.c
 * @brief   LED灯模块驱动实现
 * @details 通过GPIO BSP完成LED逻辑状态到实际GPIO电平的映射，并提供对象式访问接口。
 */
#include "led_driver.h"
#include <string.h>

static void LedBindMethods(Led_t *led);
static bool LedConfigIsValid(const LedInitConfig_t *config);
static GPIO_PinState LedStateToPinState(const Led_t *led, LedState_t state);
static LedState_t LedPinStateToState(const Led_t *led, GPIO_PinState pin_state);
static void LedSyncData(Led_t *led);

/**
 * @brief   初始化LED对象
 * @param   led LED对象指针
 * @param   config LED初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool LedInit(Led_t *led, const LedInitConfig_t *config)
{
    GPIO_Init_Config_s gpio_config;

    if ((led == NULL) || (config == NULL))
        return false;

    LedBindMethods(led);
    if (led->initialized)
        return true;

    led->init_config = *config;
    if (!LedConfigIsValid(&led->init_config))
        return false;

    memset(&led->data, 0, sizeof(led->data));

    memset(&gpio_config, 0, sizeof(gpio_config));
    gpio_config.GPIOx               = led->init_config.GPIOx;
    gpio_config.GPIO_Pin            = led->init_config.GPIO_Pin;
    gpio_config.pin_state           = LedStateToPinState(led, led->init_config.init_state);
    gpio_config.exti_mode           = GPIO_EXTI_MODE_NONE;
    gpio_config.gpio_model_callback = NULL;
    gpio_config.id                  = led->init_config.id;

    led->gpio = GPIORegister(&gpio_config);
    if (led->gpio == NULL)
        return false;

    led->initialized = true;
    LedWrite(led, led->init_config.init_state);
    return true;
}

/**
 * @brief   点亮LED
 * @param   led LED对象指针
 */
void LedOn(Led_t *led)
{
    LedWrite(led, LED_STATE_ON);
}

/**
 * @brief   熄灭LED
 * @param   led LED对象指针
 */
void LedOff(Led_t *led)
{
    LedWrite(led, LED_STATE_OFF);
}

/**
 * @brief   翻转LED状态
 * @param   led LED对象指针
 */
void LedToggle(Led_t *led)
{
    if ((led == NULL) || (!led->initialized))
        return;

    GPIOToggel(led->gpio);
    led->data.toggle_count++;
    LedSyncData(led);
}

/**
 * @brief   写入LED逻辑状态
 * @param   led LED对象指针
 * @param   state 目标LED状态
 */
void LedWrite(Led_t *led, LedState_t state)
{
    GPIO_PinState pin_state;

    if ((led == NULL) || (!led->initialized))
        return;

    pin_state = LedStateToPinState(led, state);
    if (pin_state == GPIO_PIN_SET)
        GPIOSet(led->gpio);
    else
        GPIOReset(led->gpio);

    LedSyncData(led);
}

/**
 * @brief   读取LED逻辑状态
 * @param   led LED对象指针
 * @return  LedState_t 当前LED状态
 */
LedState_t LedRead(Led_t *led)
{
    if ((led == NULL) || (!led->initialized))
        return LED_STATE_OFF;

    LedSyncData(led);
    return led->data.state;
}

/**
 * @brief   获取LED运行数据
 * @param   led LED对象指针
 * @param   data 数据输出目标
 */
void LedGetData(Led_t *led, LedData_t *data)
{
    if ((led == NULL) || (data == NULL))
        return;

    if (led->initialized)
        LedSyncData(led);

    memcpy(data, &led->data, sizeof(*data));
}

/**
 * @brief   绑定对象方法
 * @param   led LED对象指针
 */
static void LedBindMethods(Led_t *led)
{
    if (led == NULL)
        return;

    led->init     = LedInit;
    led->on       = LedOn;
    led->off      = LedOff;
    led->toggle   = LedToggle;
    led->write    = LedWrite;
    led->read     = LedRead;
    led->get_data = LedGetData;
}

/**
 * @brief   校验LED初始化配置
 * @param   config LED初始化配置
 * @return  bool 合法返回true，否则返回false
 */
static bool LedConfigIsValid(const LedInitConfig_t *config)
{
    if ((config == NULL) || (config->GPIOx == NULL) || (config->GPIO_Pin == 0U))
        return false;

    if ((config->active_state != GPIO_PIN_SET) && (config->active_state != GPIO_PIN_RESET))
        return false;

    if ((config->init_state != LED_STATE_OFF) && (config->init_state != LED_STATE_ON))
        return false;

    return true;
}

/**
 * @brief   将LED逻辑状态转换为GPIO实际电平
 * @param   led LED对象指针
 * @param   state LED逻辑状态
 * @return  GPIO_PinState GPIO实际电平
 */
static GPIO_PinState LedStateToPinState(const Led_t *led, LedState_t state)
{
    GPIO_PinState active_state;

    if (led == NULL)
        return GPIO_PIN_RESET;

    active_state = led->init_config.active_state;
    if (state == LED_STATE_ON)
        return active_state;

    return (active_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

/**
 * @brief   将GPIO实际电平转换为LED逻辑状态
 * @param   led LED对象指针
 * @param   pin_state GPIO实际电平
 * @return  LedState_t LED逻辑状态
 */
static LedState_t LedPinStateToState(const Led_t *led, GPIO_PinState pin_state)
{
    if (led == NULL)
        return LED_STATE_OFF;

    return (pin_state == led->init_config.active_state) ? LED_STATE_ON : LED_STATE_OFF;
}

/**
 * @brief   同步LED运行数据
 * @param   led LED对象指针
 */
static void LedSyncData(Led_t *led)
{
    if ((led == NULL) || (led->gpio == NULL))
        return;

    led->data.pin_state = GPIORead(led->gpio);
    led->data.state     = LedPinStateToState(led, led->data.pin_state);
}
