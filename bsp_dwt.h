#pragma once

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

static inline float DWT_GetDeltaT(uint32_t *last_tick)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint32_t delta = now - *last_tick;
    *last_tick = now;
    return (float)delta * ((float)portTICK_PERIOD_MS / 1000.0f);
}
