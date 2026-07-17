#include "bsp_gpio.h"
#include "string.h"
#include "stdlib.h"

static uint8_t idx;
static GPIOInstance *gpio_instance[GPIO_MX_DEVICE_NUM] = {NULL};

/**
 * @brief GPIO中断回调函数,根据端口和引脚找到对应的GPIOInstance并调用模块回调函数
 * @note MSPM0中断服务函数应传入实际触发的GPIO端口和引脚掩码
 *       端口中断状态可通过DL_GPIO_getPendingInterrupt获取
 * @param GPIOx 发生中断的GPIO端口
 * @param GPIO_Pin 发生中断的GPIO_Pin
 */
void GPIOIRQHandler(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    // 如有必要,可以根据pinstate和GPIORead来判断是上升沿还是下降沿/rise&fall等
    GPIOInstance *gpio;
    for (size_t i = 0; i < idx; i++)
    {
        gpio = gpio_instance[i];
        if ((gpio != NULL) &&
            (gpio->GPIOx == GPIOx) &&
            ((gpio->GPIO_Pin & GPIO_Pin) != 0U) &&
            (gpio->gpio_model_callback != NULL))
        {
            gpio->gpio_model_callback(gpio);
            return;
        }
    }
}

GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config)
{
    GPIOInstance *ins;

    if ((GPIO_config == NULL) ||
        (GPIO_config->GPIOx == NULL) ||
        (GPIO_config->GPIO_Pin == 0U) ||
        (idx >= GPIO_MX_DEVICE_NUM))
    {
        return NULL;
    }

    ins = (GPIOInstance *)malloc(sizeof(GPIOInstance));
    if (ins == NULL)
        return NULL;

    memset(ins, 0, sizeof(GPIOInstance));

    ins->GPIOx     = GPIO_config->GPIOx;
    ins->GPIO_Pin  = GPIO_config->GPIO_Pin;
    ins->pin_state = GPIO_config->pin_state;
    ins->exti_mode = GPIO_config->exti_mode;
    ins->id = GPIO_config->id;
    ins->gpio_model_callback = GPIO_config->gpio_model_callback;
    gpio_instance[idx++] = ins;
    return ins;
}

// ----------------- GPIO API -----------------
// 都是对HAL的形式上的封装,后续考虑增加GPIO state变量,可以直接读取state

void GPIOToggel(GPIOInstance *_instance)
{
    HAL_GPIO_TogglePin(_instance->GPIOx, _instance->GPIO_Pin);
}

void GPIOSet(GPIOInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_SET);
}

void GPIOReset(GPIOInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_RESET);
}

GPIO_PinState GPIORead(GPIOInstance *_instance)
{
    return HAL_GPIO_ReadPin(_instance->GPIOx, _instance->GPIO_Pin);
}
