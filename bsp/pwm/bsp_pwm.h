/**
 * @file    bsp_pwm.h
 * @brief   PWM设备底层驱动接口声明
 * @details 提供PWM通道的注册、配置（周期、占空比）及其控制接口。
 */
#pragma once

#include "mspm0_hal_compat.h"
#include <stdint.h>

#define PWM_DEVICE_CNT 16U

/**
 * @brief  PWM实例结构体
 * @note   包含对应PWM通道的所有运行状态和配置信息
 */
typedef struct pwm_ins_temp
{
    TIM_HandleTypeDef *htim;                    // 定时器句柄
    uint32_t channel;                           // 定时器通道 (如 TIM_CHANNEL_1)
    uint32_t tclk;                              // 定时器所在APB总线的时钟频率
    float period;                               // PWM周期 (单位秒)
    float dutyratio;                            // PWM占空比 (0.0~1.0)
    void (*callback)(struct pwm_ins_temp *);    // PWM周期中断或其他触发对应的回调函数
    void *id;                                   // 用户自定义ID，可用来保存额外的上下文
} PWMInstance;

/**
 * @brief  PWM设备初始化配置结构体
 * @note   用于在注册PWM实例时传递用户配置
 */
typedef struct
{
    TIM_HandleTypeDef *htim;            // 定时器句柄
    uint32_t channel;                   // 定时器通道
    float period;                       // 期望的初始周期 (单位秒)
    float dutyratio;                    // 期望的初始占空比 (0.0~1.0)
    void (*callback)(PWMInstance *);    // 回调函数
    void *id;                           // 用户自定义ID
} PWM_Init_Config_s;

/**
 * @brief   注册并初始化并启动一路PWM通道
 * @param   config PWM初始化配置结构体指针
 * @return  PWMInstance* 成功返回实例指针，失败返回NULL
 */
PWMInstance *PWMRegister(PWM_Init_Config_s *config);

/**
 * @brief   启动PWM输出
 * @param   pwm PWM实例指针
 * @note    若实例未初始化/指针为空，函数直接返回
 */
void PWMStart(PWMInstance *pwm);

/**
 * @brief   停止PWM输出
 * @param   pwm PWM实例指针
 * @note    若实例未初始化/指针为空，函数直接返回
 */
void PWMStop(PWMInstance *pwm);

/**
 * @brief   设置PWM占空比
 * @param   pwm PWM实例指针
 * @param   dutyratio 目标占空比 (0.0~1.0)
 * @note    若实例未初始化/指针为空，函数直接返回
 */
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);

/**
 * @brief   通过高电平持续时间设置PWM占空比
 * @param   pwm PWM实例指针
 * @param   pulse_width 目标高电平持续时间 (单位秒)
 * @note    若实例未初始化/指针为空，函数直接返回
 */
void PWMSetPulseWidth(PWMInstance *pwm, float pulse_width);

/**
 * @brief   设置PWM总周期
 * @param   pwm PWM实例指针
 * @param   period 目标总周期 (单位秒)
 * @note    若实例未初始化/指针为空，函数直接返回
 */
void PWMSetPeriod(PWMInstance *pwm, float period);

/**
 * @brief   启动DMA方式的PWM脉冲输出
 * @param   pwm PWM实例指针
 * @param   pData DMA源数据指针
 * @param   Size 源数据大小
 * @note    若实例未初始化/指针为空，函数直接返回
 */
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size);

/**
 * @brief   获取当前的PWM占空比
 * @param   pwm PWM实例指针
 * @return  float 当前的占空比 (0.0~1.0)
 */
float PWMGetDutyRatio(PWMInstance *pwm);

/**
 * @brief   获取当前的PWM周期
 * @param   pwm PWM实例指针
 * @return  float 当前的周期 (单位秒)
 */
float PWMGetPeriod(PWMInstance *pwm);
