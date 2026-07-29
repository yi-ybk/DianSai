/**
 * @file    hc12_freertos.c
 * @brief   Thread-safe HC-12 UART implementation for FreeRTOS tasks
 * @brief   适用于 FreeRTOS 任务的线程安全 HC-12 串口实现
 */
#include "hc12_freertos.h"

#include "hc12_config.h"
#include "hc12_uart.h"

#include <semphr.h>
#include <string.h>
#include <task.h>

/* UART3 calls FreeRTOS FromISR APIs, so its numeric priority must not be above
 * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 * UART3 会调用 FreeRTOS FromISR 接口，因此其中断优先级数值不能小于
 * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY。 */
_Static_assert(
    HC12_UART_IRQ_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
    "HC12 UART IRQ priority is too high for FreeRTOS FromISR APIs");

static StaticSemaphore_t s_tx_mutex_storage;
static StaticSemaphore_t s_rx_mutex_storage;
static StaticSemaphore_t s_rx_event_storage;
static SemaphoreHandle_t s_tx_mutex;
static SemaphoreHandle_t s_rx_mutex;
static SemaphoreHandle_t s_rx_event;
static volatile bool s_initialized;

static volatile uint32_t s_tx_bytes;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_read_timeouts;

static void Hc12RtosNotifyRxFromIsr(void *context);
static Hc12RtosStatus Hc12RtosCheckTaskContext(void);
static TickType_t Hc12RtosRemainingTicks(
    TickType_t start, TickType_t timeout_ticks);
static Hc12RtosStatus Hc12RtosReadInternal(
    uint8_t *data,
    size_t length,
    size_t *received,
    TickType_t timeout_ticks,
    bool exact);
static void Hc12RtosRecordTx(size_t count);
static void Hc12RtosRecordRx(size_t count);
static void Hc12RtosRecordTimeout(void);

bool Hc12RtosInit(void)
{
    if (s_initialized)
        return true;

    s_tx_mutex = xSemaphoreCreateMutexStatic(&s_tx_mutex_storage);
    s_rx_mutex = xSemaphoreCreateMutexStatic(&s_rx_mutex_storage);
    s_rx_event = xSemaphoreCreateBinaryStatic(&s_rx_event_storage);
    if ((s_tx_mutex == NULL) ||
        (s_rx_mutex == NULL) ||
        (s_rx_event == NULL))
    {
        return false;
    }

    s_tx_bytes = 0U;
    s_rx_bytes = 0U;
    s_read_timeouts = 0U;
    s_initialized = true;

    Hc12SetRxNotifyFromIsr(Hc12RtosNotifyRxFromIsr, NULL);
    Hc12Init();
    return true;
}

bool Hc12RtosIsInitialized(void)
{
    return s_initialized && Hc12IsInitialized();
}

Hc12RtosStatus Hc12RtosWrite(
    const uint8_t *data, size_t length, TickType_t timeout_ticks)
{
    Hc12RtosStatus status;

    if ((data == NULL) || (length == 0U))
        return HC12_RTOS_INVALID_ARGUMENT;

    status = Hc12RtosCheckTaskContext();
    if (status != HC12_RTOS_OK)
        return status;

    if (xSemaphoreTake(s_tx_mutex, timeout_ticks) != pdTRUE)
        return HC12_RTOS_TIMEOUT;

    if (!Hc12Write(data, length))
        status = HC12_RTOS_DRIVER_ERROR;
    else
        Hc12RtosRecordTx(length);

    (void)xSemaphoreGive(s_tx_mutex);
    return status;
}

Hc12RtosStatus Hc12RtosWriteString(
    const char *text, TickType_t timeout_ticks)
{
    if ((text == NULL) || (*text == '\0'))
        return HC12_RTOS_INVALID_ARGUMENT;

    return Hc12RtosWrite(
        (const uint8_t *)text, strlen(text), timeout_ticks);
}

Hc12RtosStatus Hc12RtosRead(
    uint8_t *data,
    size_t capacity,
    size_t *received,
    TickType_t timeout_ticks)
{
    return Hc12RtosReadInternal(
        data, capacity, received, timeout_ticks, false);
}

Hc12RtosStatus Hc12RtosReadExact(
    uint8_t *data,
    size_t length,
    size_t *received,
    TickType_t timeout_ticks)
{
    return Hc12RtosReadInternal(
        data, length, received, timeout_ticks, true);
}

Hc12RtosStatus Hc12RtosReadByte(
    uint8_t *value, TickType_t timeout_ticks)
{
    size_t received;

    if (value == NULL)
        return HC12_RTOS_INVALID_ARGUMENT;

    return Hc12RtosReadExact(value, 1U, &received, timeout_ticks);
}

size_t Hc12RtosAvailable(void)
{
    if (!Hc12RtosIsInitialized())
        return 0U;

    return Hc12Available();
}

Hc12RtosStatus Hc12RtosFlushRx(TickType_t timeout_ticks)
{
    Hc12RtosStatus status = Hc12RtosCheckTaskContext();

    if (status != HC12_RTOS_OK)
        return status;

    if (xSemaphoreTake(s_rx_mutex, timeout_ticks) != pdTRUE)
        return HC12_RTOS_TIMEOUT;

    Hc12FlushRx();
    while (xSemaphoreTake(s_rx_event, 0U) == pdTRUE)
        ;

    (void)xSemaphoreGive(s_rx_mutex);
    return HC12_RTOS_OK;
}

bool Hc12RtosGetStats(Hc12RtosStats *stats)
{
    if (!Hc12RtosIsInitialized() || (stats == NULL))
        return false;

    taskENTER_CRITICAL();
    stats->tx_bytes = s_tx_bytes;
    stats->rx_bytes = s_rx_bytes;
    stats->read_timeouts = s_read_timeouts;
    taskEXIT_CRITICAL();

    stats->rx_dropped_bytes = Hc12GetOverflowCount();
    return true;
}

void Hc12RtosResetStats(void)
{
    if (!Hc12RtosIsInitialized())
        return;

    taskENTER_CRITICAL();
    s_tx_bytes = 0U;
    s_rx_bytes = 0U;
    s_read_timeouts = 0U;
    taskEXIT_CRITICAL();

    (void)Hc12GetAndClearOverflowCount();
}

static void Hc12RtosNotifyRxFromIsr(void *context)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    (void)context;
    if (!s_initialized ||
        (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED))
    {
        return;
    }

    (void)xSemaphoreGiveFromISR(
        s_rx_event, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static Hc12RtosStatus Hc12RtosCheckTaskContext(void)
{
    if (!Hc12RtosIsInitialized())
        return HC12_RTOS_NOT_INITIALIZED;

    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
        return HC12_RTOS_SCHEDULER_NOT_RUNNING;

    return HC12_RTOS_OK;
}

static TickType_t Hc12RtosRemainingTicks(
    TickType_t start, TickType_t timeout_ticks)
{
    TickType_t elapsed;

    if (timeout_ticks == portMAX_DELAY)
        return portMAX_DELAY;

    elapsed = xTaskGetTickCount() - start;
    if (elapsed >= timeout_ticks)
        return 0U;

    return timeout_ticks - elapsed;
}

static Hc12RtosStatus Hc12RtosReadInternal(
    uint8_t *data,
    size_t length,
    size_t *received,
    TickType_t timeout_ticks,
    bool exact)
{
    Hc12RtosStatus status;
    TickType_t start;
    TickType_t remaining;
    size_t total = 0U;

    if ((data == NULL) || (received == NULL) || (length == 0U))
        return HC12_RTOS_INVALID_ARGUMENT;

    *received = 0U;
    status = Hc12RtosCheckTaskContext();
    if (status != HC12_RTOS_OK)
        return status;

    start = xTaskGetTickCount();
    if (xSemaphoreTake(s_rx_mutex, timeout_ticks) != pdTRUE)
    {
        Hc12RtosRecordTimeout();
        return HC12_RTOS_TIMEOUT;
    }

    status = HC12_RTOS_OK;
    for (;;)
    {
        total += Hc12Read(data + total, length - total);
        if ((total > 0U) && (!exact || (total == length)))
            break;

        remaining = Hc12RtosRemainingTicks(start, timeout_ticks);
        if ((remaining == 0U) ||
            (xSemaphoreTake(s_rx_event, remaining) != pdTRUE))
        {
            status = HC12_RTOS_TIMEOUT;
            break;
        }
    }

    (void)xSemaphoreGive(s_rx_mutex);
    *received = total;
    if (total > 0U)
        Hc12RtosRecordRx(total);
    if (status == HC12_RTOS_TIMEOUT)
        Hc12RtosRecordTimeout();

    return status;
}

static void Hc12RtosRecordTx(size_t count)
{
    taskENTER_CRITICAL();
    s_tx_bytes += (uint32_t)count;
    taskEXIT_CRITICAL();
}

static void Hc12RtosRecordRx(size_t count)
{
    taskENTER_CRITICAL();
    s_rx_bytes += (uint32_t)count;
    taskEXIT_CRITICAL();
}

static void Hc12RtosRecordTimeout(void)
{
    taskENTER_CRITICAL();
    s_read_timeouts++;
    taskEXIT_CRITICAL();
}
