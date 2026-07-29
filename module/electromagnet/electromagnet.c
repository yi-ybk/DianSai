/**
 * @file    electromagnet.c
 * @brief   电磁铁MOS驱动模块控制实现
 * @details 使用SysConfig配置的PB1控制高电平有效的MOS驱动模块。
 */
#include "electromagnet.h"

#include "ti_msp_dl_config.h"

#include <ti/driverlib/driverlib.h>

/**
 * @brief 初始化电磁铁控制，并确保电磁铁处于关闭状态
 * @note  调用前必须先执行SYSCFG_DL_init()
 */
void ElectromagnetInit(void)
{
    DL_GPIO_clearPins(GPIO_ELECTROMAGNET_PORT,
                      GPIO_ELECTROMAGNET_CONTROL_B01_PIN);
    DL_GPIO_enableOutput(GPIO_ELECTROMAGNET_PORT,
                         GPIO_ELECTROMAGNET_CONTROL_B01_PIN);
}

/**
 * @brief 打开电磁铁
 */
void ElectromagnetOn(void)
{
    DL_GPIO_setPins(GPIO_ELECTROMAGNET_PORT,
                    GPIO_ELECTROMAGNET_CONTROL_B01_PIN);
}

/**
 * @brief 关闭电磁铁
 */
void ElectromagnetOff(void)
{
    DL_GPIO_clearPins(GPIO_ELECTROMAGNET_PORT,
                      GPIO_ELECTROMAGNET_CONTROL_B01_PIN);
}

/**
 * @brief 设置电磁铁开关状态
 * @param enabled true为打开，false为关闭
 */
void ElectromagnetSet(bool enabled)
{
    if (enabled)
        ElectromagnetOn();
    else
        ElectromagnetOff();
}

/**
 * @brief 翻转电磁铁开关状态
 */
void ElectromagnetToggle(void)
{
    if (ElectromagnetIsOn())
        ElectromagnetOff();
    else
        ElectromagnetOn();
}

/**
 * @brief 查询当前控制输出状态
 * @return true表示控制引脚为高电平，false表示控制引脚为低电平
 */
bool ElectromagnetIsOn(void)
{
    return (DL_GPIO_readPins(GPIO_ELECTROMAGNET_PORT,
                             GPIO_ELECTROMAGNET_CONTROL_B01_PIN) != 0U);
}
