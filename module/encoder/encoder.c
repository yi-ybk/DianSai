/**
 * @file    encoder.c
 * @brief   编码器驱动实现
 * @details 负责将底层TIM编码器模式封装成可复用对象，并提供数据同步与方向判断。
 */
#include "encoder.h"
#include <string.h>

static void EncoderBindMethods(Encoder_t *encoder);
static bool EncoderConfigIsValid(const EncoderInitConfig_t *config);
static bool EncoderChannelIsValid(uint32_t channel);
static void EncoderNormalizeConfig(EncoderInitConfig_t *config);
static void EncoderResetSpeedWindow(Encoder_t *encoder);
static void EncoderUpdateSpeedWindow(Encoder_t *encoder, int32_t delta, float dt_s);
static bool EncoderInitSoftwareGpio(Encoder_t *encoder);
static uint8_t EncoderReadSoftwareState(const Encoder_t *encoder);
static void EncoderSoftwareGpioCallback(GPIOInstance *gpio);

static const int8_t g_encoder_transition_table[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

/**
 * @brief   初始化编码器对象
 * @param   encoder 编码器对象指针
 * @param   config 初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool EncoderInit(Encoder_t *encoder, const EncoderInitConfig_t *config)
{
    TIM_Init_Config_s tim_config;

    if ((encoder == NULL) || (config == NULL))
        return false;

    EncoderBindMethods(encoder);
    if (encoder->initialized)
        return true;

    encoder->init_config = *config;
    EncoderNormalizeConfig(&encoder->init_config);
    if (!EncoderConfigIsValid(&encoder->init_config))
        return false;

    memset(&encoder->data, 0, sizeof(encoder->data));

    if (encoder->init_config.mode == ENCODER_MODE_SOFTWARE_GPIO)
    {
        if (!EncoderInitSoftwareGpio(encoder))
            return false;
    }
    else
    {
        memset(&tim_config, 0, sizeof(tim_config));
        tim_config.htim       = encoder->init_config.htim;
        tim_config.mode       = TIM_MODE_ENCODER;
        tim_config.start_mode = encoder->init_config.auto_start ? TIM_START_POLLING : TIM_START_NONE;
        tim_config.channel    = encoder->init_config.channel;
        tim_config.id         = encoder;

        encoder->tim = TIMRegister(&tim_config);
        if (encoder->tim == NULL)
            return false;
    }

    encoder->initialized  = true;
    encoder->data.enabled = encoder->init_config.auto_start;
    EncoderReset(encoder);

    return true;
}

/**
 * @brief   启动编码器计数
 */
bool EncoderStart(Encoder_t *encoder)
{
    uint32_t primask;

    if ((encoder == NULL) || (!encoder->initialized))
        return false;

    if (encoder->init_config.mode == ENCODER_MODE_SOFTWARE_GPIO)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        encoder->software_state = EncoderReadSoftwareState(encoder);
        encoder->software_last_count = encoder->software_raw_count;
        encoder->data.enabled = true;
        if (primask == 0U)
            __enable_irq();
    }
    else
    {
        if (!TIMStart(encoder->tim))
            return false;

        encoder->data.enabled = true;
    }
    return true;
}

/**
 * @brief   停止编码器计数
 */
bool EncoderStop(Encoder_t *encoder)
{
    uint32_t primask;

    if ((encoder == NULL) || (!encoder->initialized))
        return false;

    if (encoder->init_config.mode == ENCODER_MODE_SOFTWARE_GPIO)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        encoder->data.enabled = false;
        if (primask == 0U)
            __enable_irq();
    }
    else
    {
        if (!TIMStop(encoder->tim))
            return false;

        encoder->data.enabled = false;
    }

    encoder->data.delta     = 0;
    encoder->data.direction = ENCODER_DIR_STOP;
    encoder->data.raw_speed_cps = 0.0f;
    encoder->data.speed_cps = 0.0f;
    encoder->data.speed_rps = 0.0f;
    EncoderResetSpeedWindow(encoder);
    return true;
}

/**
 * @brief   根据底层定时器计数更新编码器运行数据
 * @param   dt_s 两次更新之间的时间间隔，单位秒
 */
void EncoderUpdate(Encoder_t *encoder, float dt_s)
{
    uint32_t raw_count;
    uint32_t transition_errors;
    uint32_t primask;
    int32_t delta;

    if ((encoder == NULL) || (!encoder->initialized))
        return;

    if (!encoder->data.enabled)
    {
        encoder->data.delta     = 0;
        encoder->data.direction = ENCODER_DIR_STOP;
        encoder->data.raw_speed_cps = 0.0f;
        encoder->data.speed_cps = 0.0f;
        encoder->data.speed_rps = 0.0f;
        return;
    }

    if (encoder->init_config.mode == ENCODER_MODE_SOFTWARE_GPIO)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        raw_count = encoder->software_raw_count;
        transition_errors = encoder->software_transition_errors;
        if (primask == 0U)
            __enable_irq();

        delta = (int32_t)(raw_count - encoder->software_last_count);
        encoder->software_last_count = raw_count;
        encoder->data.transition_error_count = transition_errors;
    }
    else
    {
        raw_count = TIMGetCounter(encoder->tim);
        delta     = TIMGetDelta(encoder->tim, raw_count);
    }

    if (encoder->init_config.reversed)
        delta = -delta;

    encoder->data.raw_count = raw_count;
    encoder->data.delta     = delta;
    encoder->data.count    += delta;

    if (delta > 0)
        encoder->data.direction = ENCODER_DIR_FORWARD;
    else if (delta < 0)
        encoder->data.direction = ENCODER_DIR_REVERSE;
    else
        encoder->data.direction = ENCODER_DIR_STOP;

    if (dt_s > 0.0f)
    {
        encoder->data.raw_speed_cps = (float)delta / dt_s;
        EncoderUpdateSpeedWindow(encoder, delta, dt_s);
        encoder->data.speed_cps = encoder->speed_dt_sum > 0.0f ?
                                  (float)encoder->speed_delta_sum / encoder->speed_dt_sum :
                                  encoder->data.raw_speed_cps;
    }
    else
    {
        encoder->data.raw_speed_cps = 0.0f;
        encoder->data.speed_cps = 0.0f;
    }

    if (encoder->init_config.counts_per_rev > 0.0f)
        encoder->data.speed_rps = encoder->data.speed_cps / encoder->init_config.counts_per_rev;
    else
        encoder->data.speed_rps = 0.0f;
}

/**
 * @brief   复位编码器累计计数和速度信息
 */
void EncoderReset(Encoder_t *encoder)
{
    uint32_t primask;

    if ((encoder == NULL) || (!encoder->initialized))
        return;

    if (encoder->init_config.mode == ENCODER_MODE_SOFTWARE_GPIO)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        encoder->software_raw_count = 0U;
        encoder->software_last_count = 0U;
        encoder->software_transition_errors = 0U;
        encoder->software_state = EncoderReadSoftwareState(encoder);
        if (primask == 0U)
            __enable_irq();
    }
    else
    {
        TIMSetCounter(encoder->tim, 0U);
    }

    encoder->data.count     = 0;
    encoder->data.delta     = 0;
    encoder->data.raw_count = 0U;
    encoder->data.direction = ENCODER_DIR_STOP;
    encoder->data.raw_speed_cps = 0.0f;
    encoder->data.speed_cps = 0.0f;
    encoder->data.speed_rps = 0.0f;
    encoder->data.transition_error_count = 0U;
    EncoderResetSpeedWindow(encoder);
}

/**
 * @brief   获取编码器累计计数
 */
int32_t EncoderGetCount(Encoder_t *encoder)
{
    if (encoder == NULL)
        return 0;

    return encoder->data.count;
}

/**
 * @brief   获取编码器当前方向
 */
EncoderDirection_t EncoderGetDirection(Encoder_t *encoder)
{
    if (encoder == NULL)
        return ENCODER_DIR_STOP;

    return encoder->data.direction;
}

/**
 * @brief   获取编码器全部运行数据
 */
void EncoderGetData(Encoder_t *encoder, EncoderData_t *data)
{
    if ((encoder == NULL) || (data == NULL))
        return;

    memcpy(data, &encoder->data, sizeof(*data));
}

/**
 * @brief   绑定对象方法指针
 */
static void EncoderBindMethods(Encoder_t *encoder)
{
    if (encoder == NULL)
        return;

    encoder->init          = EncoderInit;
    encoder->start         = EncoderStart;
    encoder->stop          = EncoderStop;
    encoder->update        = EncoderUpdate;
    encoder->reset         = EncoderReset;
    encoder->get_count     = EncoderGetCount;
    encoder->get_direction = EncoderGetDirection;
    encoder->get_data      = EncoderGetData;
}

/**
 * @brief   校验编码器初始化配置是否合法
 */
static bool EncoderConfigIsValid(const EncoderInitConfig_t *config)
{
    if (config == NULL)
        return false;

    if (config->mode == ENCODER_MODE_SOFTWARE_GPIO)
    {
        if ((config->phase_a.GPIOx == NULL) ||
            (config->phase_b.GPIOx == NULL) ||
            (config->phase_a.GPIO_Pin == 0U) ||
            (config->phase_b.GPIO_Pin == 0U))
        {
            return false;
        }

        if ((config->phase_a.GPIOx == config->phase_b.GPIOx) &&
            (config->phase_a.GPIO_Pin == config->phase_b.GPIO_Pin))
        {
            return false;
        }

        return true;
    }

    if ((config->mode != ENCODER_MODE_TIMER_QEI) || (config->htim == NULL))
        return false;

    if (!EncoderChannelIsValid(config->channel))
        return false;

    return true;
}

/**
 * @brief   校验编码器通道配置是否合法
 */
static bool EncoderChannelIsValid(uint32_t channel)
{
    return ((channel == TIM_CHANNEL_ALL) ||
            (channel == TIM_CHANNEL_1) ||
            (channel == TIM_CHANNEL_2));
}

/**
 * @brief   归一化编码器配置，补充默认值
 */
static void EncoderNormalizeConfig(EncoderInitConfig_t *config)
{
    if (config == NULL)
        return;

    if (config->channel == 0U)
        config->channel = TIM_CHANNEL_ALL;

    if (config->speed_window_samples == 0U)
        config->speed_window_samples = 1U;
    else if (config->speed_window_samples > ENCODER_SPEED_WINDOW_MAX_SAMPLES)
        config->speed_window_samples = ENCODER_SPEED_WINDOW_MAX_SAMPLES;
}

static void EncoderResetSpeedWindow(Encoder_t *encoder)
{
    if (encoder == NULL)
        return;

    memset(encoder->speed_delta_history, 0, sizeof(encoder->speed_delta_history));
    memset(encoder->speed_dt_history, 0, sizeof(encoder->speed_dt_history));
    encoder->speed_delta_sum = 0;
    encoder->speed_dt_sum = 0.0f;
    encoder->speed_window_index = 0U;
    encoder->speed_window_count = 0U;
}

static void EncoderUpdateSpeedWindow(Encoder_t *encoder, int32_t delta, float dt_s)
{
    uint8_t window_samples;
    uint8_t index;

    if ((encoder == NULL) || (dt_s <= 0.0f))
        return;

    window_samples = encoder->init_config.speed_window_samples;
    index = encoder->speed_window_index;
    if (encoder->speed_window_count >= window_samples)
    {
        encoder->speed_delta_sum -= encoder->speed_delta_history[index];
        encoder->speed_dt_sum -= encoder->speed_dt_history[index];
    }
    else
    {
        encoder->speed_window_count++;
    }

    encoder->speed_delta_history[index] = delta;
    encoder->speed_dt_history[index] = dt_s;
    encoder->speed_delta_sum += delta;
    encoder->speed_dt_sum += dt_s;

    index++;
    if (index >= window_samples)
        index = 0U;
    encoder->speed_window_index = index;
}

/**
 * @brief 注册软件编码器的A/B相GPIO中断回调
 */
static bool EncoderInitSoftwareGpio(Encoder_t *encoder)
{
    GPIO_Init_Config_s gpio_config;

    memset(&gpio_config, 0, sizeof(gpio_config));
    gpio_config.GPIOx = encoder->init_config.phase_a.GPIOx;
    gpio_config.GPIO_Pin = encoder->init_config.phase_a.GPIO_Pin;
    gpio_config.exti_mode = GPIO_EXTI_MODE_RISING_FALLING;
    gpio_config.gpio_model_callback = EncoderSoftwareGpioCallback;
    gpio_config.id = encoder;
    encoder->phase_a_gpio = GPIORegister(&gpio_config);
    if (encoder->phase_a_gpio == NULL)
        return false;

    gpio_config.GPIOx = encoder->init_config.phase_b.GPIOx;
    gpio_config.GPIO_Pin = encoder->init_config.phase_b.GPIO_Pin;
    encoder->phase_b_gpio = GPIORegister(&gpio_config);
    if (encoder->phase_b_gpio == NULL)
        return false;

    encoder->software_raw_count = 0U;
    encoder->software_last_count = 0U;
    encoder->software_transition_errors = 0U;
    encoder->software_state = EncoderReadSoftwareState(encoder);
    return true;
}

/**
 * @brief 读取A/B相并编码为二位状态
 */
static uint8_t EncoderReadSoftwareState(const Encoder_t *encoder)
{
    uint8_t phase_a;
    uint8_t phase_b;

    phase_a = (GPIORead(encoder->phase_a_gpio) == GPIO_PIN_SET) ? 1U : 0U;
    phase_b = (GPIORead(encoder->phase_b_gpio) == GPIO_PIN_SET) ? 1U : 0U;
    return (uint8_t)((phase_a << 1U) | phase_b);
}

/**
 * @brief GPIO双边沿中断中的正交状态查表解码
 */
static void EncoderSoftwareGpioCallback(GPIOInstance *gpio)
{
    Encoder_t *encoder;
    uint8_t previous_state;
    uint8_t current_state;
    uint8_t table_index;
    int8_t step;

    if ((gpio == NULL) || (gpio->id == NULL))
        return;

    encoder = (Encoder_t *)gpio->id;
    if ((!encoder->initialized) || (!encoder->data.enabled))
        return;

    previous_state = encoder->software_state;
    current_state = EncoderReadSoftwareState(encoder);
    table_index = (uint8_t)((previous_state << 2U) | current_state);
    step = g_encoder_transition_table[table_index];

    if ((step == 0) && (current_state != previous_state))
        encoder->software_transition_errors++;
    else if (step > 0)
        encoder->software_raw_count++;
    else if (step < 0)
        encoder->software_raw_count--;

    encoder->software_state = current_state;
}
