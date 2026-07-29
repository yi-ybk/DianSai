#pragma once

#include <stddef.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

typedef TaskHandle_t osThreadId_t;
typedef enum {
    osOK             = 0,
    osError          = -1,
    osErrorTimeout   = -2,
    osErrorResource  = -3,
    osErrorParameter = -4,
    osErrorNoMemory  = -5,
    osErrorISR       = -6,
} osStatus_t;
typedef enum {
    osPriorityError       = -1,
    osPriorityNone        = 0,
    osPriorityIdle        = 1,
    osPriorityLow         = 8,
    osPriorityBelowNormal = 16,
    osPriorityNormal      = 24,
    osPriorityAboveNormal = 32,
    osPriorityHigh        = 40,
    osPriorityRealtime    = 48,
    osPriorityISR         = 56,
} osPriority_t;
typedef struct {
    const char *name;
    osPriority_t priority;
    void *cb_mem;
    uint32_t cb_size;
    void *stack_mem;
    uint32_t stack_size;
} osThreadAttr_t;

#define osFlagsWaitAny 0U
#define osWaitForever portMAX_DELAY

static inline UBaseType_t osPriorityToFreeRTOS(osPriority_t priority)
{
    uint32_t level;

    if ((priority == osPriorityNone) ||
        (priority < osPriorityIdle) ||
        (priority >= osPriorityISR)) {
        priority = osPriorityNormal;
    }

    level = (priority == osPriorityIdle) ? 0U : ((uint32_t)priority / 8U);
    return (UBaseType_t)((level * (configMAX_PRIORITIES - 1U) + 3U) / 6U);
}

static inline osThreadId_t osThreadNew(void (*entry)(void *), void *argument, const osThreadAttr_t *attributes)
{
    TaskHandle_t task = NULL;
    uint16_t stack = (attributes == NULL) ? configMINIMAL_STACK_SIZE : (uint16_t)(attributes->stack_size / sizeof(StackType_t));
    osPriority_t cmsis_priority = (attributes == NULL) ? osPriorityNormal : attributes->priority;
    UBaseType_t priority = osPriorityToFreeRTOS(cmsis_priority);

    if ((attributes != NULL) &&
        (attributes->cb_mem != NULL) &&
        (attributes->cb_size >= sizeof(StaticTask_t)) &&
        (attributes->stack_mem != NULL) &&
        (attributes->stack_size >= sizeof(StackType_t))) {
        return xTaskCreateStatic(entry,
                                 attributes->name,
                                 stack,
                                 argument,
                                 priority,
                                 (StackType_t *)attributes->stack_mem,
                                 (StaticTask_t *)attributes->cb_mem);
    }

    if (xTaskCreate(entry, (attributes == NULL) ? "task" : attributes->name, stack, argument, priority, &task) != pdPASS) return NULL;
    return task;
}

static inline osStatus_t osDelay(uint32_t ticks)
{
    if (ticks == 0U) return osErrorParameter;
    vTaskDelay((TickType_t)ticks);
    return osOK;
}

static inline uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    uint32_t value = 0U;
    (void)options;
    (void)xTaskNotifyWait(0U, flags, &value, timeout);
    return value;
}
