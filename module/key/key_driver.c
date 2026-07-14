/**
 * @file    key_driver.c
 * @brief   按键模块驱动实现
 * @details 通过GPIO BSP外部中断回调识别按键事件，并向上层提供可自定义事件回调。
 */
#include "key_driver.h"
#include <string.h>

static void KeyBindMethods(Key_t *key);
static bool KeyConfigIsValid(const KeyInitConfig_t *config);
static bool KeyExtiModeIsValid(GPIO_EXTI_MODE_e mode);
static KeyState_t KeyPinStateToState(const Key_t *key, GPIO_PinState pin_state);
static KeyEvent_t KeyStateToEvent(KeyState_t state);
static void KeySyncData(Key_t *key);
static void KeyGpioCallback(GPIOInstance *gpio);
static void KeyCopyData(Key_t *key, KeyData_t *data);

/**
 * @brief   初始化按键对象
 * @param   key 按键对象指针
 * @param   config 按键初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool KeyInit(Key_t *key, const KeyInitConfig_t *config)
{
    GPIO_Init_Config_s gpio_config;

    if ((key == NULL) || (config == NULL))
        return false;

    KeyBindMethods(key);
    if (key->initialized)
        return true;

    key->init_config = *config;
    if (!KeyConfigIsValid(&key->init_config))
        return false;

    memset(&key->data, 0, sizeof(key->data));
    key->data.last_event      = KEY_EVENT_NONE;
    key->data.last_event_tick = HAL_GetTick() - key->init_config.debounce_ms;

    memset(&gpio_config, 0, sizeof(gpio_config));
    gpio_config.GPIOx               = key->init_config.GPIOx;
    gpio_config.GPIO_Pin            = key->init_config.GPIO_Pin;
    gpio_config.pin_state           = GPIO_PIN_RESET;
    gpio_config.exti_mode           = key->init_config.exti_mode;
    gpio_config.gpio_model_callback = KeyGpioCallback;
    gpio_config.id                  = key;

    key->gpio = GPIORegister(&gpio_config);
    if (key->gpio == NULL)
        return false;

    key->initialized = true;
    KeySyncData(key);
    return true;
}

/**
 * @brief   读取按键逻辑状态
 * @param   key 按键对象指针
 * @return  KeyState_t 当前按键状态
 */
KeyState_t KeyRead(Key_t *key)
{
    if ((key == NULL) || (!key->initialized))
        return KEY_STATE_RELEASED;

    KeySyncData(key);
    return key->data.state;
}

/**
 * @brief   获取最近一次按键事件
 * @param   key 按键对象指针
 * @return  KeyEvent_t 最近一次按键事件
 */
KeyEvent_t KeyGetLastEvent(Key_t *key)
{
    if ((key == NULL) || (!key->initialized))
        return KEY_EVENT_NONE;

    return key->data.last_event;
}

/**
 * @brief   清除最近一次按键事件
 * @param   key 按键对象指针
 */
void KeyClearEvent(Key_t *key)
{
    uint32_t primask;

    if ((key == NULL) || (!key->initialized))
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    key->data.last_event = KEY_EVENT_NONE;
    if (primask == 0U)
        __enable_irq();
}

/**
 * @brief   设置按键事件回调函数
 * @param   key 按键对象指针
 * @param   callback 事件回调函数
 * @param   context 用户上下文
 */
void KeySetCallback(Key_t *key, KeyEventCallback_t callback, void *context)
{
    uint32_t primask;

    if ((key == NULL) || (!key->initialized))
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    key->init_config.event_callback = callback;
    key->init_config.event_context  = context;
    if (primask == 0U)
        __enable_irq();
}

/**
 * @brief   获取按键运行数据
 * @param   key 按键对象指针
 * @param   data 数据输出目标
 */
void KeyGetData(Key_t *key, KeyData_t *data)
{
    if ((key == NULL) || (data == NULL))
        return;

    if (key->initialized)
        KeySyncData(key);

    KeyCopyData(key, data);
}

/**
 * @brief   绑定对象方法
 * @param   key 按键对象指针
 */
static void KeyBindMethods(Key_t *key)
{
    if (key == NULL)
        return;

    key->init           = KeyInit;
    key->read           = KeyRead;
    key->get_last_event = KeyGetLastEvent;
    key->clear_event    = KeyClearEvent;
    key->set_callback   = KeySetCallback;
    key->get_data       = KeyGetData;
}

/**
 * @brief   校验按键初始化配置
 * @param   config 按键初始化配置
 * @return  bool 合法返回true，否则返回false
 */
static bool KeyConfigIsValid(const KeyInitConfig_t *config)
{
    if ((config == NULL) || (config->GPIOx == NULL) || (config->GPIO_Pin == 0U))
        return false;

    if ((config->active_state != GPIO_PIN_SET) && (config->active_state != GPIO_PIN_RESET))
        return false;

    if (!KeyExtiModeIsValid(config->exti_mode))
        return false;

    return true;
}

/**
 * @brief   校验外部中断触发模式
 * @param   mode 外部中断触发模式
 * @return  bool 合法返回true，否则返回false
 */
static bool KeyExtiModeIsValid(GPIO_EXTI_MODE_e mode)
{
    if ((mode == GPIO_EXTI_MODE_RISING) ||
        (mode == GPIO_EXTI_MODE_FALLING) ||
        (mode == GPIO_EXTI_MODE_RISING_FALLING))
    {
        return true;
    }

    return false;
}

/**
 * @brief   将GPIO实际电平转换为按键逻辑状态
 * @param   key 按键对象指针
 * @param   pin_state GPIO实际电平
 * @return  KeyState_t 按键逻辑状态
 */
static KeyState_t KeyPinStateToState(const Key_t *key, GPIO_PinState pin_state)
{
    if (key == NULL)
        return KEY_STATE_RELEASED;

    return (pin_state == key->init_config.active_state) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
}

/**
 * @brief   将按键逻辑状态转换为按键事件
 * @param   state 按键逻辑状态
 * @return  KeyEvent_t 按键事件
 */
static KeyEvent_t KeyStateToEvent(KeyState_t state)
{
    return (state == KEY_STATE_PRESSED) ? KEY_EVENT_PRESS : KEY_EVENT_RELEASE;
}

/**
 * @brief   同步按键运行数据
 * @param   key 按键对象指针
 */
static void KeySyncData(Key_t *key)
{
    GPIO_PinState pin_state;

    if ((key == NULL) || (key->gpio == NULL))
        return;

    pin_state = GPIORead(key->gpio);
    key->data.state = KeyPinStateToState(key, pin_state);
}

/**
 * @brief   GPIO外部中断回调入口
 * @param   gpio 触发中断的GPIO对象
 */
static void KeyGpioCallback(GPIOInstance *gpio)
{
    Key_t *key;
    KeyEvent_t event;
    KeyState_t previous_state;
    uint32_t now_tick;

    if ((gpio == NULL) || (gpio->id == NULL))
        return;

    key = (Key_t *)gpio->id;
    if (!key->initialized)
        return;

    now_tick = HAL_GetTick();
    if ((key->init_config.debounce_ms > 0U) &&
        ((now_tick - key->data.last_event_tick) < key->init_config.debounce_ms))
    {
        return;
    }

    previous_state = key->data.state;
    KeySyncData(key);
    if (key->data.state == previous_state)
        return;

    event = KeyStateToEvent(key->data.state);

    key->data.last_event      = event;
    key->data.last_event_tick = now_tick;
    key->data.event_count++;
    if (event == KEY_EVENT_PRESS)
        key->data.press_count++;
    else if (event == KEY_EVENT_RELEASE)
        key->data.release_count++;

    if (key->init_config.event_callback != NULL)
        key->init_config.event_callback(key, event, key->init_config.event_context);
}

/**
 * @brief   复制按键运行数据
 * @param   key 按键对象指针
 * @param   data 数据输出目标
 */
static void KeyCopyData(Key_t *key, KeyData_t *data)
{
    uint32_t primask;

    if ((key == NULL) || (data == NULL))
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    *data = key->data;
    if (primask == 0U)
        __enable_irq();
}
