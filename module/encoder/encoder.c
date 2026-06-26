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

    memset(&tim_config, 0, sizeof(tim_config));
    tim_config.htim       = encoder->init_config.htim;
    tim_config.mode       = TIM_MODE_ENCODER;
    tim_config.start_mode = encoder->init_config.auto_start ? TIM_START_POLLING : TIM_START_NONE;
    tim_config.channel    = encoder->init_config.channel;
    tim_config.id         = encoder;

    encoder->tim = TIMRegister(&tim_config);
    if (encoder->tim == NULL)
        return false;

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
    if ((encoder == NULL) || (!encoder->initialized))
        return false;

    if (!TIMStart(encoder->tim))
        return false;

    encoder->data.enabled = true;
    return true;
}

/**
 * @brief   停止编码器计数
 */
bool EncoderStop(Encoder_t *encoder)
{
    if ((encoder == NULL) || (!encoder->initialized))
        return false;

    if (!TIMStop(encoder->tim))
        return false;

    encoder->data.enabled   = false;
    encoder->data.delta     = 0;
    encoder->data.direction = ENCODER_DIR_STOP;
    encoder->data.speed_cps = 0.0f;
    encoder->data.speed_rps = 0.0f;
    return true;
}

/**
 * @brief   根据底层定时器计数更新编码器运行数据
 * @param   dt_s 两次更新之间的时间间隔，单位秒
 */
void EncoderUpdate(Encoder_t *encoder, float dt_s)
{
    uint32_t raw_count;
    int32_t delta;

    if ((encoder == NULL) || (!encoder->initialized))
        return;

    if (!encoder->data.enabled)
    {
        encoder->data.delta     = 0;
        encoder->data.direction = ENCODER_DIR_STOP;
        encoder->data.speed_cps = 0.0f;
        encoder->data.speed_rps = 0.0f;
        return;
    }

    raw_count = TIMGetCounter(encoder->tim);
    delta     = TIMGetDelta(encoder->tim, raw_count);
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
        encoder->data.speed_cps = (float)delta / dt_s;
    else
        encoder->data.speed_cps = 0.0f;

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
    if ((encoder == NULL) || (encoder->tim == NULL))
        return;

    TIMSetCounter(encoder->tim, 0U);
    encoder->data.count     = 0;
    encoder->data.delta     = 0;
    encoder->data.raw_count = 0U;
    encoder->data.direction = ENCODER_DIR_STOP;
    encoder->data.speed_cps = 0.0f;
    encoder->data.speed_rps = 0.0f;
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
    if ((config == NULL) || (config->htim == NULL))
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
}
