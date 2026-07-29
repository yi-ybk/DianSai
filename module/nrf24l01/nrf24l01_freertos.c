/**
 * @file    nrf24l01_freertos.c
 * @brief   Thread-safe nRF24L01+ implementation for FreeRTOS tasks
 * @brief   适用于 FreeRTOS 任务的线程安全 nRF24L01+ 实现
 */
#include "nrf24l01_freertos.h"

#include <stddef.h>
#include <string.h>
#include <task.h>

_Static_assert(configSUPPORT_STATIC_ALLOCATION == 1,
               "nRF24 RTOS wrapper requires static allocation support");

static Nrf24Result_t Nrf24RtosCheckTaskContext(
    const Nrf24Rtos_t *radio);
static Nrf24Result_t Nrf24RtosTakeDevice(
    Nrf24Rtos_t *radio, TickType_t timeout_ticks);
static TickType_t Nrf24RtosRemainingTicks(
    TickType_t start, TickType_t timeout_ticks);
static void Nrf24RtosRecordTx(
    Nrf24Rtos_t *radio, uint8_t length, Nrf24Result_t result);
static void Nrf24RtosRecordRx(
    Nrf24Rtos_t *radio, uint8_t length);
static void Nrf24RtosRecordReceiveTimeout(Nrf24Rtos_t *radio);
static void Nrf24RtosRecordLockTimeout(Nrf24Rtos_t *radio);

void Nrf24RtosGetTianmengxingConfig(Nrf24InitConfig_t *config)
{
    Nrf24SimpleGetTianmengxingConfig(config);
}

Nrf24Result_t Nrf24RtosInit(Nrf24Rtos_t *radio,
                             const Nrf24InitConfig_t *config,
                             bool start_receiving)
{
    Nrf24Result_t result;

    if ((radio == NULL) || (config == NULL))
        return NRF24_RESULT_INVALID_ARGUMENT;

    memset(radio, 0, sizeof(*radio));
    radio->device_mutex =
        xSemaphoreCreateMutexStatic(&radio->device_mutex_storage);
    radio->receive_mutex =
        xSemaphoreCreateMutexStatic(&radio->receive_mutex_storage);
    if ((radio->device_mutex == NULL) ||
        (radio->receive_mutex == NULL))
    {
        return NRF24_RESULT_IO_ERROR;
    }

    result = Nrf24SimpleInit(
        &radio->simple, config, start_receiving);
    if (result != NRF24_RESULT_OK)
        return result;

    radio->initialized = true;
    return NRF24_RESULT_OK;
}

bool Nrf24RtosIsInitialized(const Nrf24Rtos_t *radio)
{
    return (radio != NULL) &&
           radio->initialized &&
           Nrf24SimpleIsInitialized(&radio->simple);
}

Nrf24Result_t Nrf24RtosSend(Nrf24Rtos_t *radio,
                             const uint8_t *data,
                             uint8_t length,
                             uint32_t radio_timeout_ms,
                             TickType_t lock_timeout_ticks,
                             bool resume_receive)
{
    Nrf24Result_t result;

    if ((data == NULL) || (length == 0U) ||
        (length > NRF24_MAX_PAYLOAD_SIZE))
    {
        return NRF24_RESULT_INVALID_ARGUMENT;
    }

    result = Nrf24RtosCheckTaskContext(radio);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24RtosTakeDevice(radio, lock_timeout_ticks);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24SimpleSend(
        &radio->simple,
        data,
        length,
        radio_timeout_ms,
        resume_receive);
    (void)xSemaphoreGive(radio->device_mutex);

    Nrf24RtosRecordTx(radio, length, result);
    return result;
}

Nrf24Result_t Nrf24RtosReceive(Nrf24Rtos_t *radio,
                                uint8_t *data,
                                uint8_t capacity,
                                uint8_t *length,
                                uint8_t *pipe,
                                TickType_t timeout_ticks)
{
    Nrf24Result_t result;
    TickType_t start;
    TickType_t remaining;

    if ((data == NULL) || (capacity == 0U) || (length == NULL))
        return NRF24_RESULT_INVALID_ARGUMENT;

    *length = 0U;
    result = Nrf24RtosCheckTaskContext(radio);
    if (result != NRF24_RESULT_OK)
        return result;

    start = xTaskGetTickCount();
    if (xSemaphoreTake(radio->receive_mutex, timeout_ticks) != pdTRUE)
    {
        Nrf24RtosRecordLockTimeout(radio);
        return NRF24_RESULT_LOCK_TIMEOUT;
    }

    result = NRF24_RESULT_NO_DATA;
    for (;;)
    {
        remaining = Nrf24RtosRemainingTicks(start, timeout_ticks);
        result = Nrf24RtosTakeDevice(radio, remaining);
        if (result != NRF24_RESULT_OK)
            break;

        /* Use the simple driver's non-blocking path. Waiting is performed with
         * vTaskDelay below so the CPU and radio mutex are released.
         * 使用便捷驱动的非阻塞路径；等待由下方 vTaskDelay 完成，以便释放
         * CPU 和无线模块互斥量。 */
        result = Nrf24SimpleReceive(
            &radio->simple, data, capacity, length, pipe, 0U);
        (void)xSemaphoreGive(radio->device_mutex);

        if (result != NRF24_RESULT_NO_DATA)
            break;
        if (timeout_ticks == 0U)
            break;

        remaining = Nrf24RtosRemainingTicks(start, timeout_ticks);
        if (remaining == 0U)
        {
            result = NRF24_RESULT_TIMEOUT;
            break;
        }

        vTaskDelay(1U);
    }

    (void)xSemaphoreGive(radio->receive_mutex);

    if (result == NRF24_RESULT_OK)
        Nrf24RtosRecordRx(radio, *length);
    else if (result == NRF24_RESULT_TIMEOUT)
        Nrf24RtosRecordReceiveTimeout(radio);

    return result;
}

Nrf24Result_t Nrf24RtosStartReceive(Nrf24Rtos_t *radio,
                                     bool clear_pending,
                                     TickType_t lock_timeout_ticks)
{
    Nrf24Result_t result = Nrf24RtosCheckTaskContext(radio);

    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24RtosTakeDevice(radio, lock_timeout_ticks);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24SimpleStartReceive(
        &radio->simple, clear_pending);
    (void)xSemaphoreGive(radio->device_mutex);
    return result;
}

Nrf24Result_t Nrf24RtosStopReceive(
    Nrf24Rtos_t *radio, TickType_t lock_timeout_ticks)
{
    Nrf24Result_t result = Nrf24RtosCheckTaskContext(radio);

    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24RtosTakeDevice(radio, lock_timeout_ticks);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24SimpleStopReceive(&radio->simple);
    (void)xSemaphoreGive(radio->device_mutex);
    return result;
}

Nrf24Result_t Nrf24RtosPowerDown(
    Nrf24Rtos_t *radio, TickType_t lock_timeout_ticks)
{
    Nrf24Result_t result = Nrf24RtosCheckTaskContext(radio);

    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24RtosTakeDevice(radio, lock_timeout_ticks);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24SimplePowerDown(&radio->simple);
    (void)xSemaphoreGive(radio->device_mutex);
    return result;
}

Nrf24Result_t Nrf24RtosDataAvailable(Nrf24Rtos_t *radio,
                                      bool *available,
                                      TickType_t lock_timeout_ticks)
{
    Nrf24Result_t result;

    if (available == NULL)
        return NRF24_RESULT_INVALID_ARGUMENT;
    *available = false;

    result = Nrf24RtosCheckTaskContext(radio);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24RtosTakeDevice(radio, lock_timeout_ticks);
    if (result != NRF24_RESULT_OK)
        return result;

    *available = Nrf24SimpleDataAvailable(&radio->simple);
    (void)xSemaphoreGive(radio->device_mutex);
    return NRF24_RESULT_OK;
}

Nrf24Result_t Nrf24RtosCheckConnection(Nrf24Rtos_t *radio,
                                        bool *connected,
                                        TickType_t lock_timeout_ticks)
{
    Nrf24Result_t result;

    if (connected == NULL)
        return NRF24_RESULT_INVALID_ARGUMENT;
    *connected = false;

    result = Nrf24RtosCheckTaskContext(radio);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24RtosTakeDevice(radio, lock_timeout_ticks);
    if (result != NRF24_RESULT_OK)
        return result;

    *connected = Nrf24CheckConnection(&radio->simple.device);
    (void)xSemaphoreGive(radio->device_mutex);
    return NRF24_RESULT_OK;
}

bool Nrf24RtosGetStats(
    const Nrf24Rtos_t *radio, Nrf24RtosStats_t *stats)
{
    if (!Nrf24RtosIsInitialized(radio) || (stats == NULL))
        return false;

    taskENTER_CRITICAL();
    stats->tx_packets = radio->tx_packets;
    stats->tx_bytes = radio->tx_bytes;
    stats->tx_failures = radio->tx_failures;
    stats->rx_packets = radio->rx_packets;
    stats->rx_bytes = radio->rx_bytes;
    stats->receive_timeouts = radio->receive_timeouts;
    stats->lock_timeouts = radio->lock_timeouts;
    taskEXIT_CRITICAL();
    return true;
}

void Nrf24RtosResetStats(Nrf24Rtos_t *radio)
{
    if (!Nrf24RtosIsInitialized(radio))
        return;

    taskENTER_CRITICAL();
    radio->tx_packets = 0U;
    radio->tx_bytes = 0U;
    radio->tx_failures = 0U;
    radio->rx_packets = 0U;
    radio->rx_bytes = 0U;
    radio->receive_timeouts = 0U;
    radio->lock_timeouts = 0U;
    taskEXIT_CRITICAL();
}

static Nrf24Result_t Nrf24RtosCheckTaskContext(
    const Nrf24Rtos_t *radio)
{
    if (!Nrf24RtosIsInitialized(radio))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
        return NRF24_RESULT_RTOS_NOT_RUNNING;

    return NRF24_RESULT_OK;
}

static Nrf24Result_t Nrf24RtosTakeDevice(
    Nrf24Rtos_t *radio, TickType_t timeout_ticks)
{
    if (xSemaphoreTake(radio->device_mutex, timeout_ticks) == pdTRUE)
        return NRF24_RESULT_OK;

    Nrf24RtosRecordLockTimeout(radio);
    return NRF24_RESULT_LOCK_TIMEOUT;
}

static TickType_t Nrf24RtosRemainingTicks(
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

static void Nrf24RtosRecordTx(
    Nrf24Rtos_t *radio, uint8_t length, Nrf24Result_t result)
{
    taskENTER_CRITICAL();
    if (result == NRF24_RESULT_OK)
    {
        radio->tx_packets++;
        radio->tx_bytes += length;
    }
    else
    {
        radio->tx_failures++;
    }
    taskEXIT_CRITICAL();
}

static void Nrf24RtosRecordRx(
    Nrf24Rtos_t *radio, uint8_t length)
{
    taskENTER_CRITICAL();
    radio->rx_packets++;
    radio->rx_bytes += length;
    taskEXIT_CRITICAL();
}

static void Nrf24RtosRecordReceiveTimeout(Nrf24Rtos_t *radio)
{
    taskENTER_CRITICAL();
    radio->receive_timeouts++;
    taskEXIT_CRITICAL();
}

static void Nrf24RtosRecordLockTimeout(Nrf24Rtos_t *radio)
{
    taskENTER_CRITICAL();
    radio->lock_timeouts++;
    taskEXIT_CRITICAL();
}
