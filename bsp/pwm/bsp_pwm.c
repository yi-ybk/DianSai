/**
 * @file    bsp_pwm.c
 * @brief   PWM设备底层驱动实现
 * @details 封装并抽象了STM32 HAL库的PWM底层具体行为，通过对象实例管理多路PWM。
 */
#include "bsp_pwm.h"
#include <string.h>

/* -------------------- 静态变量区 -------------------- */
/** @brief 已注册的PWM实例总数 */
static uint8_t idx;
/** @brief 预分配的全局PWM实例静态内存池 */
static PWMInstance pwm_instance_pool[PWM_DEVICE_CNT];
/** @brief 全局注册的PWM实例指针数组，用于中断中查表触发回调 */
static PWMInstance *pwm_instance[PWM_DEVICE_CNT];

static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim);
static float PWMClamp(float value, float min, float max);
static uint8_t PWMChannelIsValid(uint32_t channel);

/**
 * @brief   PWM DMA传输完成回调函数（HAL库回调）
 * @details 当PWM DMA输出完成时，HAL库会自动调用该函数；
 *          函数内部遍历已注册的PWM实例，匹配触发中断的定时器+通道，调用对应实例的回调函数。
 * @param   htim 发生中断的定时器句柄（HAL库传入）
 * @note    1. 需确保该函数被HAL库正确调用（需在stm32xxxx_it.c中保留TIM相关中断映射）；
 *          2. 一次中断仅匹配一个实例（定时器+通道唯一），匹配后立即返回；
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < idx; i++)
    {   // 来自同一个定时器的中断且通道相同
        if ((pwm_instance[i] != NULL) &&
            (pwm_instance[i]->htim == htim) &&
            (htim->Channel == (1U << (pwm_instance[i]->channel / 4U))))
        {
            if (pwm_instance[i]->callback != NULL)  // 如果有回调函数
                pwm_instance[i]->callback(pwm_instance[i]);

            return; // 一次只能有一个通道的中断,所以直接返回
        }
    }
}


PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    PWMInstance *pwm;

    if ((config == NULL) ||
        (config->htim == NULL) ||
        (!PWMChannelIsValid(config->channel)) ||
        (config->period <= 0.0f))
    {
        return NULL;
    }

    for (uint8_t i = 0; i < idx; i++)
    {
        if ((pwm_instance[i] != NULL) &&
            (pwm_instance[i]->htim == config->htim) &&
            (pwm_instance[i]->channel == config->channel))
        {
            return pwm_instance[i]; // 返回已存在的实例
        }
    }

    if (idx >= PWM_DEVICE_CNT)  // 超过最大实例数,考虑增加或查看是否有内存泄漏
        return NULL;

    pwm = &pwm_instance_pool[idx];
    memset(pwm, 0, sizeof(*pwm));

    pwm->htim      = config->htim;
    pwm->channel   = config->channel;
    pwm->period    = config->period;
    pwm->dutyratio = PWMClamp(config->dutyratio, 0.0f, 1.0f);
    pwm->callback  = config->callback;
    pwm->id        = config->id;
    pwm->tclk      = PWMSelectTclk(pwm->htim);    // 计算定时器工作时钟，失败则返回NULL
    if (pwm->tclk == 0U)
        return NULL;

    PWMSetPeriod(pwm, pwm->period);
    PWMSetDutyRatio(pwm, pwm->dutyratio);
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);

    pwm_instance[idx++] = pwm;
    return pwm;
}

void PWMStart(PWMInstance *pwm)
{
    if (pwm == NULL)
        return;

    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
}

void PWMStop(PWMInstance *pwm)
{
    if (pwm == NULL)
        return;

    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

void PWMSetPeriod(PWMInstance *pwm, float period)
{
    float timer_freq;
    uint32_t ticks;

    if ((pwm == NULL) || (pwm->htim == NULL) || (period <= 0.0f) || (pwm->tclk == 0U))
        return;

    timer_freq = (float)pwm->tclk / (float)(pwm->htim->Init.Prescaler + 1U);
    ticks = (uint32_t)(period * timer_freq);
    if (ticks == 0U)
        ticks = 1U;

    pwm->period = period;
    __HAL_TIM_SetAutoreload(pwm->htim, ticks - 1U);
}

void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio)
{
    uint32_t compare;
    uint32_t period_ticks;

    if ((pwm == NULL) || (pwm->htim == NULL))
        return;

    dutyratio = PWMClamp(dutyratio, 0.0f, 1.0f);
    period_ticks = pwm->htim->Init.Period + 1U;
    if (dutyratio <= 0.0f)
        compare = pwm->htim->Init.Period;
    else if (dutyratio >= 1.0f)
        compare = 0U;
    else
        compare = (uint32_t)((1.0f - dutyratio) * (float)period_ticks) - 1U;

    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, compare);
    pwm->dutyratio = dutyratio;
}

void PWMSetPulseWidth(PWMInstance *pwm, float pulse_width)
{
    if ((pwm == NULL) || (pwm->period <= 0.0f))
        return;

    PWMSetDutyRatio(pwm, pulse_width / pwm->period);
}

void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size)
{
    if ((pwm == NULL) || (pData == NULL) || (Size == 0U))
        return;

    HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
}

float PWMGetDutyRatio(PWMInstance *pwm)
{
    if (pwm == NULL)
        return 0.0f;

    return pwm->dutyratio;
}

float PWMGetPeriod(PWMInstance *pwm)
{
    if (pwm == NULL)
        return 0.0f;

    return pwm->period;
}

/**
 * @brief   根据定时器基地址推算该定时器挂载在APB1还是APB2，从而获取其工作时钟频率
 * @param   htim 定时器句柄
 * @return  uint32_t 当前定时器的工作时钟(Tclk)频率(Hz)
 */
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim)
{
    return (htim == NULL) ? 0U : htim->clock_hz;
}

/**
 * @brief   浮点型数据限幅保护
 * @param   value 输入值
 * @param   min   下限
 * @param   max   上限
 * @return  float 返回被限制在[min, max]区间的值
 */
static float PWMClamp(float value, float min, float max)
{
    if (value < min)
        return min;

    if (value > max)
        return max;

    return value;
}

/**
 * @brief   判断待初始化的PWM通道宏定义是否合法
 * @param   channel HAL库定义的通道宏
 * @return  uint8_t 合法返回1，非法返回0
 */
static uint8_t PWMChannelIsValid(uint32_t channel)
{
    return ((channel == TIM_CHANNEL_1) ||
            (channel == TIM_CHANNEL_2) ||
            (channel == TIM_CHANNEL_3) ||
            (channel == TIM_CHANNEL_4));
}
