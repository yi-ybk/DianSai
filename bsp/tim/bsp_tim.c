/**
 * @file    bsp_tim.c
 * @brief   定时器底层驱动实现
 * @details 负责管理定时器实例、启停控制、编码器计数增量计算以及周期回调分发。
 */
#include "bsp_tim.h"
#include <string.h>

/* -------------------- 静态资源区 -------------------- */
static uint8_t idx;
static TIMInstance tim_instance_pool[TIM_DEVICE_CNT];
static TIMInstance *tim_instances[TIM_DEVICE_CNT];

/* ------------------ 内部静态函数声明 ------------------ */
static uint8_t TIMConfigIsValid(const TIM_Init_Config_s *config);
static uint32_t TIMNormalizeChannel(TIMMode_t mode, uint32_t channel);
static TIMInstance *TIMFindByHandle(TIM_HandleTypeDef *htim);

/**
 * @brief   注册一个定时器实例
 * @param   config 定时器初始化配置
 * @return  TIMInstance* 注册成功返回实例指针，失败返回NULL
 */
TIMInstance *TIMRegister(TIM_Init_Config_s *config)
{
    TIMInstance *instance;
    uint32_t channel;

    if (!TIMConfigIsValid(config))
        return NULL;

    for (uint8_t i = 0; i < idx; i++)
    {
        if ((tim_instances[i] != NULL) &&
            (tim_instances[i]->htim == config->htim) &&
            (tim_instances[i]->mode == config->mode))
        {
            return tim_instances[i];
        }
    }

    if (idx >= TIM_DEVICE_CNT)
        return NULL;

    channel = TIMNormalizeChannel(config->mode, config->channel);

    instance = &tim_instance_pool[idx];
    memset(instance, 0, sizeof(*instance));
    instance->htim            = config->htim;
    instance->mode            = config->mode;
    instance->start_mode      = config->start_mode;
    instance->channel         = channel;
    instance->last_counter    = __HAL_TIM_GET_COUNTER(config->htim);
    instance->module_callback = config->module_callback;
    instance->id              = config->id;

    tim_instances[idx++] = instance;

    if (instance->start_mode != TIM_START_NONE)
    {
        if (!TIMStart(instance))
            return NULL;
    }

    return instance;
}

/**
 * @brief   启动定时器实例
 */
uint8_t TIMStart(TIMInstance *instance)
{
    if ((instance == NULL) || (instance->htim == NULL))
        return 0;

    if (instance->mode == TIM_MODE_ENCODER)
    {
        if (instance->start_mode == TIM_START_IT)
            return (HAL_TIM_Encoder_Start_IT(instance->htim, instance->channel) == HAL_OK);

        return (HAL_TIM_Encoder_Start(instance->htim, instance->channel) == HAL_OK);
    }

    if (instance->start_mode == TIM_START_IT)
        return (HAL_TIM_Base_Start_IT(instance->htim) == HAL_OK);

    return (HAL_TIM_Base_Start(instance->htim) == HAL_OK);
}

/**
 * @brief   停止定时器实例
 */
uint8_t TIMStop(TIMInstance *instance)
{
    if ((instance == NULL) || (instance->htim == NULL))
        return 0;

    if (instance->mode == TIM_MODE_ENCODER)
    {
        if (instance->start_mode == TIM_START_IT)
            return (HAL_TIM_Encoder_Stop_IT(instance->htim, instance->channel) == HAL_OK);

        return (HAL_TIM_Encoder_Stop(instance->htim, instance->channel) == HAL_OK);
    }

    if (instance->start_mode == TIM_START_IT)
        return (HAL_TIM_Base_Stop_IT(instance->htim) == HAL_OK);

    return (HAL_TIM_Base_Stop(instance->htim) == HAL_OK);
}

/**
 * @brief   获取定时器当前计数值
 */
uint32_t TIMGetCounter(TIMInstance *instance)
{
    if ((instance == NULL) || (instance->htim == NULL))
        return 0U;

    return __HAL_TIM_GET_COUNTER(instance->htim);
}

/**
 * @brief   设置定时器当前计数值
 */
void TIMSetCounter(TIMInstance *instance, uint32_t counter)
{
    if ((instance == NULL) || (instance->htim == NULL))
        return;

    __HAL_TIM_SET_COUNTER(instance->htim, counter);
    instance->last_counter = counter;
}

/**
 * @brief   获取定时器自动重装载值
 */
uint32_t TIMGetAutoReload(TIMInstance *instance)
{
    if ((instance == NULL) || (instance->htim == NULL))
        return 0U;

    return instance->htim->Instance->ARR;
}

/**
 * @brief   设置定时器自动重装载值
 */
void TIMSetAutoReload(TIMInstance *instance, uint32_t autoreload)
{
    if ((instance == NULL) || (instance->htim == NULL))
        return;

    __HAL_TIM_SET_AUTORELOAD(instance->htim, autoreload);
}

/**
 * @brief   判断定时器是否正在向下计数
 */
uint8_t TIMIsCountingDown(TIMInstance *instance)
{
    if ((instance == NULL) || (instance->htim == NULL))
        return 0;

    return (__HAL_TIM_IS_TIM_COUNTING_DOWN(instance->htim) ? 1U : 0U);
}

/**
 * @brief   计算定时器计数增量，自动处理溢出回绕
 */
int32_t TIMGetDelta(TIMInstance *instance, uint32_t current_counter)
{
    uint64_t period;
    uint64_t last_counter;
    uint64_t counter;
    int64_t delta;

    if ((instance == NULL) || (instance->htim == NULL))
        return 0;

    period       = (uint64_t)instance->htim->Init.Period + 1ULL;
    last_counter = (uint64_t)instance->last_counter;
    counter      = (uint64_t)current_counter;

    if (period == 0ULL)
        period = 0x100000000ULL;

    delta = (int64_t)counter - (int64_t)last_counter;
    if (delta > (int64_t)(period / 2ULL))
        delta -= (int64_t)period;
    else if (delta < -(int64_t)(period / 2ULL))
        delta += (int64_t)period;

    instance->last_counter = current_counter;

    if (delta > 2147483647LL)
        return 2147483647;

    if (delta < (-2147483647LL - 1LL))
        return (-2147483647 - 1);

    return (int32_t)delta;
}

/**
 * @brief   定时器周期到达回调分发入口
 */
void TIMPeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    TIMInstance *instance;

    instance = TIMFindByHandle(htim);
    if ((instance != NULL) && (instance->module_callback != NULL))
        instance->module_callback(instance);
}

/**
 * @brief   校验定时器初始化配置是否合法
 */
static uint8_t TIMConfigIsValid(const TIM_Init_Config_s *config)
{
    if ((config == NULL) || (config->htim == NULL))
        return 0;

    if ((config->mode != TIM_MODE_BASE) && (config->mode != TIM_MODE_ENCODER))
        return 0;

    if ((config->start_mode != TIM_START_NONE) &&
        (config->start_mode != TIM_START_POLLING) &&
        (config->start_mode != TIM_START_IT))
    {
        return 0;
    }

    return 1;
}

/**
 * @brief   对通道配置进行归一化处理
 */
static uint32_t TIMNormalizeChannel(TIMMode_t mode, uint32_t channel)
{
    if (mode != TIM_MODE_ENCODER)
        return 0U;

    if (channel == 0U)
        return TIM_CHANNEL_ALL;

    return channel;
}

/**
 * @brief   通过HAL句柄查找已注册的定时器实例
 */
static TIMInstance *TIMFindByHandle(TIM_HandleTypeDef *htim)
{
    if (htim == NULL)
        return NULL;

    for (uint8_t i = 0; i < idx; i++)
    {
        if ((tim_instances[i] != NULL) && (tim_instances[i]->htim == htim))
            return tim_instances[i];
    }

    return NULL;
}
