#pragma once

#include "mspm0_hal_compat.h"
#include "stdint.h"

#define GPIO_MX_DEVICE_NUM 32U

/**
 * @brief 用于判断中断来源,注意和CUBEMX中配置一致
 *
 */
typedef enum
{
    GPIO_EXTI_MODE_RISING,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
    GPIO_EXTI_MODE_NONE,
} GPIO_EXTI_MODE_e;

/**
 * @brief GPIO实例结构体定义
 *
 */
typedef struct tmpgpio
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset;not frequently used
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式 not frequently used
    uint32_t GPIO_Pin;          // MSPM0 GPIO引脚掩码，如GPIO_PIN_0
    void (*gpio_model_callback)(struct tmpgpio *); // exti中断回调函数
    void *id;                                      // 区分不同的GPIO实例
} GPIOInstance;

/**
 * @brief GPIO初始化配置结构体定义
 *
 */
typedef struct
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset not frequently used
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式 not frequently used
    uint32_t GPIO_Pin;          // MSPM0 GPIO引脚掩码，如GPIO_PIN_0、GPIO_PIN_1

    void (*gpio_model_callback)(GPIOInstance *); // exti中断回调函数
    void *id;                                    // 区分不同的GPIO实例

} GPIO_Init_Config_s;

/**
 * @brief 注册GPIO实例
 *
 * @param GPIO_config
 * @return GPIOInstance*
 */
GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config);

/**
 * @brief GPIO API,切换GPIO电平
 *
 * @param _instance
 */
void GPIOToggel(GPIOInstance *_instance);

/**
 * @brief 设置GPIO电平
 *
 * @param _instance
 */
void GPIOSet(GPIOInstance *_instance);

/**
 * @brief 复位GPIO电平
 *
 * @param _instance
 */
void GPIOReset(GPIOInstance *_instance);

/**
 * @brief 读取GPIO电平
 *
 * @param _instance
 * @return GPIO_PinState
 */
GPIO_PinState GPIORead(GPIOInstance *_instance);

/**
 * @brief MSPM0 GPIO中断服务分发函数
 * @param GPIOx 触发中断的GPIO端口
 * @param GPIO_Pin 触发中断的引脚掩码
 * @note 在具体GPIO端口中断服务函数中调用
 */
void GPIOIRQHandler(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
