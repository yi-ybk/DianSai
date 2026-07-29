/**
 * @file    nrf24l01_freertos.h
 * @brief   Thread-safe nRF24L01+ API for FreeRTOS tasks on MSPM0G3507
 * @brief   MSPM0G3507 上适用于 FreeRTOS 任务的线程安全 nRF24L01+ 接口
 */
#pragma once

#include "nrf24l01_simple.h"

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <semphr.h>

/**
 * @brief Runtime counters maintained by the FreeRTOS wrapper.
 * @brief FreeRTOS 封装层维护的运行统计。
 */
typedef struct
{
    uint32_t tx_packets;
    uint32_t tx_bytes;
    uint32_t tx_failures;
    uint32_t rx_packets;
    uint32_t rx_bytes;
    uint32_t receive_timeouts;
    uint32_t lock_timeouts;
} Nrf24RtosStats_t;

/**
 * @brief One statically allocated task-safe radio instance.
 * @brief 一个使用静态内存的任务安全无线模块实例。
 *
 * Keep this object at static or global scope and do not move or copy it after
 * initialization because FreeRTOS object handles point into this structure.
 *
 * 该对象应放在静态或全局作用域。初始化后不可移动或复制，因为 FreeRTOS
 * 对象句柄指向结构体内部的静态存储区。
 */
typedef struct
{
    Nrf24Simple_t simple;
    StaticSemaphore_t device_mutex_storage;
    StaticSemaphore_t receive_mutex_storage;
    SemaphoreHandle_t device_mutex;
    SemaphoreHandle_t receive_mutex;
    volatile uint32_t tx_packets;
    volatile uint32_t tx_bytes;
    volatile uint32_t tx_failures;
    volatile uint32_t rx_packets;
    volatile uint32_t rx_bytes;
    volatile uint32_t receive_timeouts;
    volatile uint32_t lock_timeouts;
    bool initialized;
} Nrf24Rtos_t;

/**
 * @brief Fill the verified Tianmengxing/MaixCAM radio configuration.
 * @brief 填充已验证的天猛星/MaixCAM 无线配置。
 */
void Nrf24RtosGetTianmengxingConfig(Nrf24InitConfig_t *config);

/**
 * @brief Initialize synchronization objects and the radio.
 * @brief 初始化同步对象和无线模块。
 *
 * Call once after SYSCFG_DL_init(), normally before vTaskStartScheduler().
 * The initialization performs the driver's blocking power-on delay but does
 * not allocate FreeRTOS heap memory.
 *
 * 在 SYSCFG_DL_init() 后调用一次，通常放在 vTaskStartScheduler() 之前。
 * 初始化包含驱动的阻塞上电等待，但不会使用 FreeRTOS 堆。
 */
Nrf24Result_t Nrf24RtosInit(Nrf24Rtos_t *radio,
                             const Nrf24InitConfig_t *config,
                             bool start_receiving);

/**
 * @brief Return whether the wrapper and low-level radio are initialized.
 * @brief 返回封装层和底层无线模块是否已经初始化。
 */
bool Nrf24RtosIsInitialized(const Nrf24Rtos_t *radio);

/**
 * @brief Send one complete application payload without task interleaving.
 * @brief 发送一条完整应用负载，防止多个任务操作互相交织。
 *
 * @param radio Task-safe radio object. / 任务安全无线对象。
 * @param data Application payload. / 应用负载。
 * @param length Payload length from 1 to 32. / 负载长度，范围 1 到 32。
 * @param radio_timeout_ms nRF24 transmit/ACK timeout in milliseconds.
 *                         / nRF24 发送和 ACK 超时，单位毫秒。
 * @param lock_timeout_ticks Maximum wait for exclusive device access.
 *                           / 等待设备互斥量的最大 Tick 数。
 * @param resume_receive Restore RX mode when it was active before sending.
 *                       / 发送前处于 RX 模式时，发送后恢复 RX。
 */
Nrf24Result_t Nrf24RtosSend(Nrf24Rtos_t *radio,
                             const uint8_t *data,
                             uint8_t length,
                             uint32_t radio_timeout_ms,
                             TickType_t lock_timeout_ticks,
                             bool resume_receive);

/**
 * @brief Receive one packet with a FreeRTOS Tick timeout.
 * @brief 使用 FreeRTOS Tick 超时接收一个数据包。
 *
 * timeout_ticks=0 performs one non-blocking check. portMAX_DELAY waits
 * indefinitely. The wrapper releases the device mutex between checks so a
 * transmit task can still use the radio.
 *
 * timeout_ticks=0 表示非阻塞检查一次，portMAX_DELAY 表示一直等待。封装层在
 * 两次检查之间释放设备互斥量，因此发送任务仍可使用无线模块。
 */
Nrf24Result_t Nrf24RtosReceive(Nrf24Rtos_t *radio,
                                uint8_t *data,
                                uint8_t capacity,
                                uint8_t *length,
                                uint8_t *pipe,
                                TickType_t timeout_ticks);

/**
 * @brief Enter receive mode with exclusive device access.
 * @brief 取得设备独占访问后进入接收模式。
 */
Nrf24Result_t Nrf24RtosStartReceive(Nrf24Rtos_t *radio,
                                     bool clear_pending,
                                     TickType_t lock_timeout_ticks);

/**
 * @brief Leave receive mode and remain in powered-up standby.
 * @brief 离开接收模式并保持在已上电待机状态。
 */
Nrf24Result_t Nrf24RtosStopReceive(
    Nrf24Rtos_t *radio, TickType_t lock_timeout_ticks);

/**
 * @brief Enter the nRF24L01+ power-down state.
 * @brief 使 nRF24L01+ 进入掉电状态。
 */
Nrf24Result_t Nrf24RtosPowerDown(
    Nrf24Rtos_t *radio, TickType_t lock_timeout_ticks);

/**
 * @brief Check whether the RX FIFO currently contains a packet.
 * @brief 检查 RX FIFO 当前是否存在数据包。
 */
Nrf24Result_t Nrf24RtosDataAvailable(Nrf24Rtos_t *radio,
                                      bool *available,
                                      TickType_t lock_timeout_ticks);

/**
 * @brief Check SPI communication with the nRF24L01+.
 * @brief 检查与 nRF24L01+ 的 SPI 通信是否正常。
 */
Nrf24Result_t Nrf24RtosCheckConnection(Nrf24Rtos_t *radio,
                                        bool *connected,
                                        TickType_t lock_timeout_ticks);

/**
 * @brief Copy a consistent snapshot of wrapper statistics.
 * @brief 获取封装层统计的一致快照。
 */
bool Nrf24RtosGetStats(
    const Nrf24Rtos_t *radio, Nrf24RtosStats_t *stats);

/**
 * @brief Clear all wrapper statistics.
 * @brief 清零封装层的全部统计。
 */
void Nrf24RtosResetStats(Nrf24Rtos_t *radio);
