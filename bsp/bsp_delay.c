/**
 * @file    bsp_delay.c
 * @brief   基于 CPU 周期忙等待的阻塞延时实现
 */
#include "bsp_delay.h"

#include "ti_msp_dl_config.h"

#define BSP_DELAY_CYCLES_PER_US (CPUCLK_FREQ / 1000000U)

void BSP_DelayUs(uint32_t us)
{
    const uint32_t max_us_per_call = UINT32_MAX / BSP_DELAY_CYCLES_PER_US;

    while (us > max_us_per_call)
    {
        DL_Common_delayCycles(max_us_per_call * BSP_DELAY_CYCLES_PER_US);
        us -= max_us_per_call;
    }

    if (us > 0U)
        DL_Common_delayCycles(us * BSP_DELAY_CYCLES_PER_US);
}

void BSP_DelayMs(uint32_t ms)
{
    while (ms-- > 0U)
        BSP_DelayUs(1000U);
}
