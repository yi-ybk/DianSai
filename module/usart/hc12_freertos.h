/**
 * @file    hc12_freertos.h
 * @brief   Thread-safe HC-12 UART API for FreeRTOS tasks
 * @brief   适用于 FreeRTOS 任务的线程安全 HC-12 串口接口
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <FreeRTOS.h>

/**
 * @brief Result returned by the FreeRTOS-aware HC-12 functions.
 * @brief HC-12 FreeRTOS 接口返回状态。
 */
typedef enum
{
    HC12_RTOS_OK = 0,
    HC12_RTOS_TIMEOUT,
    HC12_RTOS_NOT_INITIALIZED,
    HC12_RTOS_INVALID_ARGUMENT,
    HC12_RTOS_SCHEDULER_NOT_RUNNING,
    HC12_RTOS_DRIVER_ERROR
} Hc12RtosStatus;

/**
 * @brief Runtime counters maintained by the task-safe wrapper.
 * @brief 线程安全封装层维护的运行统计。
 */
typedef struct
{
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t read_timeouts;
    uint32_t rx_dropped_bytes;
} Hc12RtosStats;

/**
 * @brief Create static RTOS synchronization objects and initialize UART3.
 * @brief 创建静态 RTOS 同步对象并初始化 UART3。
 *
 * Call once after SYSCFG_DL_init(), normally before vTaskStartScheduler().
 * No FreeRTOS heap memory is allocated.
 *
 * 在 SYSCFG_DL_init() 之后调用一次，通常放在 vTaskStartScheduler() 之前。
 * 本函数不使用 FreeRTOS 堆。
 */
bool Hc12RtosInit(void);

/**
 * @brief Return whether the FreeRTOS wrapper has been initialized.
 * @brief 返回 FreeRTOS 封装层是否已经初始化。
 */
bool Hc12RtosIsInitialized(void);

/**
 * @brief Send one complete binary message without interleaving with other tasks.
 * @brief 发送一条完整二进制消息，避免与其他任务的数据交织。
 *
 * @param data Data to send. / 待发送数据。
 * @param length Data length. / 数据长度。
 * @param timeout_ticks Maximum wait for the transmit mutex.
 *                      / 等待发送互斥量的最大 Tick 数。
 *
 * The timeout only limits mutex acquisition. After the mutex is acquired,
 * hardware transmission is blocking until all bytes have been written.
 *
 * 超时仅限制等待互斥量的时间；取得互斥量后，硬件发送将阻塞到全部字节写完。
 */
Hc12RtosStatus Hc12RtosWrite(
    const uint8_t *data, size_t length, TickType_t timeout_ticks);

/**
 * @brief Send one null-terminated string as an indivisible task message.
 * @brief 将一个字符串作为不可交织的任务消息发送。
 */
Hc12RtosStatus Hc12RtosWriteString(
    const char *text, TickType_t timeout_ticks);

/**
 * @brief Wait for at least one byte, then return all currently available bytes.
 * @brief 等待至少一个字节，然后返回当前可读取的数据。
 *
 * @param data Destination buffer. / 目标缓冲区。
 * @param capacity Destination capacity. / 目标缓冲区容量。
 * @param received Number of bytes copied. / 实际读取字节数。
 * @param timeout_ticks Total receive timeout, including mutex wait.
 *                      / 总接收超时，包括等待接收互斥量的时间。
 */
Hc12RtosStatus Hc12RtosRead(
    uint8_t *data,
    size_t capacity,
    size_t *received,
    TickType_t timeout_ticks);

/**
 * @brief Wait until the requested number of bytes is received or timeout occurs.
 * @brief 等待指定长度的数据，或在超时后返回。
 *
 * A timeout may return partial data through received. This function keeps the
 * receive mutex for the whole operation so another task cannot take bytes from
 * the middle of the message.
 *
 * 超时时 received 可能返回部分数据。本函数在整个操作期间持有接收互斥量，
 * 防止其他任务从一条消息中间取走数据。
 */
Hc12RtosStatus Hc12RtosReadExact(
    uint8_t *data,
    size_t length,
    size_t *received,
    TickType_t timeout_ticks);

/**
 * @brief Read one byte with a timeout.
 * @brief 带超时读取一个字节。
 */
Hc12RtosStatus Hc12RtosReadByte(
    uint8_t *value, TickType_t timeout_ticks);

/**
 * @brief Return a snapshot of buffered receive bytes.
 * @brief 返回当前接收缓冲区字节数的快照。
 */
size_t Hc12RtosAvailable(void);

/**
 * @brief Clear the receive ring buffer while holding the receive mutex.
 * @brief 持有接收互斥量时清空接收环形缓冲区。
 */
Hc12RtosStatus Hc12RtosFlushRx(TickType_t timeout_ticks);

/**
 * @brief Copy current transmit, receive, timeout and overflow counters.
 * @brief 获取发送、接收、超时和溢出统计。
 */
bool Hc12RtosGetStats(Hc12RtosStats *stats);

/**
 * @brief Clear all counters maintained by the driver and wrapper.
 * @brief 清零驱动层和封装层维护的全部统计。
 */
void Hc12RtosResetStats(void);
