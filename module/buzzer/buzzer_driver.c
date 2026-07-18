/**
 * @file    buzzer_driver.c
 * @brief   蜂鸣器模块驱动实现
 * @details 有源蜂鸣器通过GPIO输出控制，无源蜂鸣器通过PWM频率和占空比控制。
 */
#include "buzzer_driver.h"
#include <string.h>

static void BuzzerBindMethods(Buzzer_t *buzzer);
static bool BuzzerConfigIsValid(const BuzzerInitConfig_t *config);
static bool BuzzerStateIsValid(BuzzerState_t state);
static GPIO_PinState BuzzerStateToPinState(const Buzzer_t *buzzer, BuzzerState_t state);

/**
 * @brief   初始化蜂鸣器对象
 * @param   buzzer 蜂鸣器对象指针
 * @param   config 蜂鸣器初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool BuzzerInit(Buzzer_t *buzzer, const BuzzerInitConfig_t *config)
{
    GPIO_Init_Config_s gpio_config;
    PWM_Init_Config_s pwm_config;

    if ((buzzer == NULL) || (config == NULL))
        return false;

    BuzzerBindMethods(buzzer);
    if (buzzer->initialized)
        return true;

    buzzer->init_config = *config;
    if (!BuzzerConfigIsValid(&buzzer->init_config))
        return false;

    buzzer->gpio = NULL;
    buzzer->pwm = NULL;
    memset(&buzzer->data, 0, sizeof(buzzer->data));
    buzzer->data.type = buzzer->init_config.type;
    buzzer->data.frequency_hz = buzzer->init_config.pwm.frequency_hz;
    buzzer->data.duty_ratio = buzzer->init_config.pwm.duty_ratio;

    if (buzzer->init_config.type == BUZZER_TYPE_ACTIVE_GPIO)
    {
        memset(&gpio_config, 0, sizeof(gpio_config));
        gpio_config.GPIOx = buzzer->init_config.gpio.GPIOx;
        gpio_config.GPIO_Pin = buzzer->init_config.gpio.GPIO_Pin;
        gpio_config.pin_state = BuzzerStateToPinState(buzzer, BUZZER_STATE_OFF);
        gpio_config.exti_mode = GPIO_EXTI_MODE_NONE;
        gpio_config.gpio_model_callback = NULL;
        gpio_config.id = buzzer->init_config.id;

        buzzer->gpio = GPIORegister(&gpio_config);
        if (buzzer->gpio == NULL)
            return false;
    }
    else
    {
        memset(&pwm_config, 0, sizeof(pwm_config));
        pwm_config.htim = buzzer->init_config.pwm.htim;
        pwm_config.channel = buzzer->init_config.pwm.channel;
        pwm_config.period = 1.0f / buzzer->init_config.pwm.frequency_hz;
        pwm_config.dutyratio = 0.0f;
        pwm_config.callback = NULL;
        pwm_config.id = buzzer->init_config.id;

        buzzer->pwm = PWMRegister(&pwm_config);
        if (buzzer->pwm == NULL)
            return false;

        PWMSetDutyRatio(buzzer->pwm, 0.0f);
        PWMStop(buzzer->pwm);
    }

    buzzer->initialized = true;
    BuzzerWrite(buzzer, buzzer->init_config.init_state);
    return true;
}

/**
 * @brief   启动蜂鸣器
 * @param   buzzer 蜂鸣器对象指针
 */
void BuzzerOn(Buzzer_t *buzzer)
{
    BuzzerWrite(buzzer, BUZZER_STATE_ON);
}

/**
 * @brief   停止蜂鸣器
 * @param   buzzer 蜂鸣器对象指针
 */
void BuzzerOff(Buzzer_t *buzzer)
{
    BuzzerWrite(buzzer, BUZZER_STATE_OFF);
}

/**
 * @brief   翻转蜂鸣器状态
 * @param   buzzer 蜂鸣器对象指针
 */
void BuzzerToggle(Buzzer_t *buzzer)
{
    if ((buzzer == NULL) || (!buzzer->initialized))
        return;

    BuzzerWrite(buzzer,
                (buzzer->data.state == BUZZER_STATE_ON) ?
                BUZZER_STATE_OFF : BUZZER_STATE_ON);
}

/**
 * @brief   写入蜂鸣器逻辑状态
 * @param   buzzer 蜂鸣器对象指针
 * @param   state 目标逻辑状态
 */
void BuzzerWrite(Buzzer_t *buzzer, BuzzerState_t state)
{
    GPIO_PinState pin_state;
    BuzzerState_t previous_state;

    if ((buzzer == NULL) || (!buzzer->initialized) || (!BuzzerStateIsValid(state)))
        return;

    previous_state = buzzer->data.state;
    if (buzzer->init_config.type == BUZZER_TYPE_ACTIVE_GPIO)
    {
        pin_state = BuzzerStateToPinState(buzzer, state);
        if (pin_state == GPIO_PIN_SET)
            GPIOSet(buzzer->gpio);
        else
            GPIOReset(buzzer->gpio);
    }
    else if (state == BUZZER_STATE_ON)
    {
        PWMSetPeriod(buzzer->pwm, 1.0f / buzzer->data.frequency_hz);
        PWMSetDutyRatio(buzzer->pwm, buzzer->data.duty_ratio);
        PWMStart(buzzer->pwm);
    }
    else
    {
        PWMSetDutyRatio(buzzer->pwm, 0.0f);
        PWMStop(buzzer->pwm);
    }

    buzzer->data.state = state;
    if ((previous_state == BUZZER_STATE_OFF) && (state == BUZZER_STATE_ON))
        buzzer->data.on_count++;
}

/**
 * @brief   设置无源蜂鸣器频率和占空比
 * @param   buzzer 蜂鸣器对象指针
 * @param   frequency_hz 目标频率(Hz)
 * @param   duty_ratio 目标占空比，取值范围0.0~1.0
 * @return  bool 设置成功返回true，有源蜂鸣器或参数非法返回false
 */
bool BuzzerSetTone(Buzzer_t *buzzer, float frequency_hz, float duty_ratio)
{
    if ((buzzer == NULL) || (!buzzer->initialized) ||
        (buzzer->init_config.type != BUZZER_TYPE_PASSIVE_PWM) ||
        (frequency_hz <= 0.0f) ||
        (duty_ratio < 0.0f) || (duty_ratio > 1.0f))
    {
        return false;
    }

    buzzer->init_config.pwm.frequency_hz = frequency_hz;
    buzzer->init_config.pwm.duty_ratio = duty_ratio;
    buzzer->data.frequency_hz = frequency_hz;
    buzzer->data.duty_ratio = duty_ratio;

    PWMSetPeriod(buzzer->pwm, 1.0f / frequency_hz);
    PWMSetDutyRatio(buzzer->pwm,
                    (buzzer->data.state == BUZZER_STATE_ON) ? duty_ratio : 0.0f);
    return true;
}

/**
 * @brief   获取蜂鸣器运行数据
 * @param   buzzer 蜂鸣器对象指针
 * @param   data 数据输出目标
 */
void BuzzerGetData(Buzzer_t *buzzer, BuzzerData_t *data)
{
    if ((buzzer == NULL) || (data == NULL))
        return;

    memcpy(data, &buzzer->data, sizeof(*data));
}

/**
 * @brief   绑定蜂鸣器对象方法
 * @param   buzzer 蜂鸣器对象指针
 */
static void BuzzerBindMethods(Buzzer_t *buzzer)
{
    if (buzzer == NULL)
        return;

    buzzer->init = BuzzerInit;
    buzzer->on = BuzzerOn;
    buzzer->off = BuzzerOff;
    buzzer->toggle = BuzzerToggle;
    buzzer->write = BuzzerWrite;
    buzzer->set_tone = BuzzerSetTone;
    buzzer->get_data = BuzzerGetData;
}

/**
 * @brief   校验蜂鸣器初始化配置
 * @param   config 蜂鸣器初始化配置
 * @return  bool 合法返回true，否则返回false
 */
static bool BuzzerConfigIsValid(const BuzzerInitConfig_t *config)
{
    if ((config == NULL) || (!BuzzerStateIsValid(config->init_state)))
        return false;

    if (config->type == BUZZER_TYPE_ACTIVE_GPIO)
    {
        if ((config->gpio.GPIOx == NULL) || (config->gpio.GPIO_Pin == 0U))
            return false;

        return ((config->gpio.active_state == GPIO_PIN_SET) ||
                (config->gpio.active_state == GPIO_PIN_RESET));
    }

    if (config->type == BUZZER_TYPE_PASSIVE_PWM)
    {
        return ((config->pwm.htim != NULL) &&
                (config->pwm.frequency_hz > 0.0f) &&
                (config->pwm.duty_ratio >= 0.0f) &&
                (config->pwm.duty_ratio <= 1.0f));
    }

    return false;
}

/**
 * @brief   校验蜂鸣器逻辑状态
 * @param   state 蜂鸣器逻辑状态
 * @return  bool 合法返回true，否则返回false
 */
static bool BuzzerStateIsValid(BuzzerState_t state)
{
    return ((state == BUZZER_STATE_OFF) || (state == BUZZER_STATE_ON));
}

/**
 * @brief   将蜂鸣器逻辑状态转换为GPIO实际电平
 * @param   buzzer 蜂鸣器对象指针
 * @param   state 蜂鸣器逻辑状态
 * @return  GPIO_PinState GPIO实际电平
 */
static GPIO_PinState BuzzerStateToPinState(const Buzzer_t *buzzer, BuzzerState_t state)
{
    GPIO_PinState active_state;

    if (buzzer == NULL)
        return GPIO_PIN_RESET;

    active_state = buzzer->init_config.gpio.active_state;
    if (state == BUZZER_STATE_ON)
        return active_state;

    return (active_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
