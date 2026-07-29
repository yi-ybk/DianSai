/**
 * @file    nrf24l01_simple.h
 * @brief   Application-level nRF24L01+ convenience API for MSPM0G3507
 * @brief   MSPM0G3507 的 nRF24L01+ 应用层便捷 API
 */
#pragma once

#include "nrf24l01.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Application-level radio object with receive-mode tracking.
 * @brief 带接收模式状态跟踪的应用层无线模块对象。
 */
typedef struct
{
    Nrf24_t device; /**< Low-level radio device. / 底层无线模块设备。 */
    bool receiving; /**< Current RX-mode state. / 当前接收模式状态。 */
} Nrf24Simple_t;

/**
 * @brief Fill the configuration verified on the Tianmengxing MSPM0G3507 board.
 * @brief 填充已在天猛星 MSPM0G3507 开发板上验证的配置。
 * @param config Destination configuration. / 目标配置结构体。
 *
 * The generated configuration uses PB23/PB22/PB21 for software SPI,
 * PB24/PB26/PB27 for CE/CSN/IRQ, channel 76, address D2:F0:F0:F0:A5,
 * 1 Mbps, -18 dBm, auto ACK and a fixed 32-byte payload.
 *
 * 生成的配置使用 PB23/PB22/PB21 作为软件 SPI，PB24/PB26/PB27 作为
 * CE/CSN/IRQ，并采用频道 76、地址 D2:F0:F0:F0:A5、1 Mbps、-18 dBm、
 * 自动应答和固定 32 字节负载。
 */
void Nrf24SimpleGetTianmengxingConfig(Nrf24InitConfig_t *config);

/**
 * @brief Initialize one radio and optionally enter receive mode.
 * @brief 初始化一个无线模块，并可选择立即进入接收模式。
 * @param radio Application-level radio object. / 应用层无线模块对象。
 * @param config Radio and board configuration. / 无线和板级配置。
 * @param start_receiving true enters RX mode after initialization.
 *                        / true 表示初始化后进入接收模式。
 * @return NRF24_RESULT_OK on success, otherwise an error code.
 *         / 成功返回 NRF24_RESULT_OK，否则返回错误码。
 */
Nrf24Result_t Nrf24SimpleInit(Nrf24Simple_t *radio,
                              const Nrf24InitConfig_t *config,
                              bool start_receiving);

/**
 * @brief Return whether the wrapper and low-level driver are initialized.
 * @brief 返回便捷封装和底层驱动是否均已初始化。
 * @param radio Application-level radio object. / 应用层无线模块对象。
 * @return true when initialized. / 已初始化时返回 true。
 */
bool Nrf24SimpleIsInitialized(const Nrf24Simple_t *radio);

/**
 * @brief Send one application payload.
 * @brief 发送一个应用层负载。
 *
 * In fixed-payload mode, data shorter than payload_size is padded with zero.
 * resume_receive restores RX mode only when the radio was receiving before
 * this call.
 *
 * 固定负载模式下，长度小于 payload_size 的数据会在尾部补零。只有模块在
 * 调用前处于接收模式时，resume_receive 才会在发送后恢复接收模式。
 *
 * @param radio Application-level radio object. / 应用层无线模块对象。
 * @param data Application data to send. / 待发送的应用数据。
 * @param length Application data length in bytes. / 应用数据长度，单位字节。
 * @param timeout_ms Transmit timeout in milliseconds. / 发送超时，单位毫秒。
 * @param resume_receive Restore the previous RX state after transmission.
 *                       / 发送后是否恢复之前的接收状态。
 * @return Transmission result. / 发送结果。
 */
Nrf24Result_t Nrf24SimpleSend(Nrf24Simple_t *radio,
                              const uint8_t *data,
                              uint8_t length,
                              uint32_t timeout_ms,
                              bool resume_receive);

/**
 * @brief Enter receive mode.
 * @brief 进入接收模式。
 * @param radio Application-level radio object. / 应用层无线模块对象。
 * @param clear_pending true discards packets already present in RX FIFO.
 *                      / true 表示丢弃 RX FIFO 中已有的数据包。
 * @return NRF24_RESULT_OK on success, otherwise an error code.
 *         / 成功返回 NRF24_RESULT_OK，否则返回错误码。
 */
Nrf24Result_t Nrf24SimpleStartReceive(Nrf24Simple_t *radio,
                                      bool clear_pending);

/**
 * @brief Leave receive mode and remain in powered-up standby mode.
 * @brief 离开接收模式，并保持在已上电待机状态。
 * @param radio Application-level radio object. / 应用层无线模块对象。
 * @return Operation result. / 操作结果。
 */
Nrf24Result_t Nrf24SimpleStopReceive(Nrf24Simple_t *radio);

/**
 * @brief Receive one payload, optionally waiting for it.
 * @brief 接收一个负载，并可选择等待数据到达。
 *
 * timeout_ms=0 performs a non-blocking check and returns
 * NRF24_RESULT_NO_DATA when no packet is ready.  A positive timeout returns
 * NRF24_RESULT_TIMEOUT when it expires.
 *
 * timeout_ms=0 表示非阻塞检查，没有数据时返回 NRF24_RESULT_NO_DATA。
 * timeout_ms 为正数时最多等待对应毫秒数，超时返回 NRF24_RESULT_TIMEOUT。
 *
 * @param radio Application-level radio object. / 应用层无线模块对象。
 * @param data Receive buffer. / 接收缓冲区。
 * @param capacity Receive-buffer capacity. / 接收缓冲区容量。
 * @param length Actual received length. / 实际接收长度。
 * @param pipe Source pipe number; may be NULL. / 来源管道号，可为 NULL。
 * @param timeout_ms Receive timeout in milliseconds. / 接收超时，单位毫秒。
 * @return Receive result. / 接收结果。
 */
Nrf24Result_t Nrf24SimpleReceive(Nrf24Simple_t *radio,
                                 uint8_t *data,
                                 uint8_t capacity,
                                 uint8_t *length,
                                 uint8_t *pipe,
                                 uint32_t timeout_ms);

/**
 * @brief Return whether at least one packet is waiting in RX FIFO.
 * @brief 返回 RX FIFO 中是否至少有一个待读取数据包。
 * @param radio Application-level radio object. / 应用层无线模块对象。
 */
bool Nrf24SimpleDataAvailable(Nrf24Simple_t *radio);

/**
 * @brief Enter the nRF24L01+ low-power state.
 * @brief 使 nRF24L01+ 进入低功耗掉电状态。
 * @param radio Application-level radio object. / 应用层无线模块对象。
 * @return Operation result. / 操作结果。
 */
Nrf24Result_t Nrf24SimplePowerDown(Nrf24Simple_t *radio);

/**
 * @brief Access the low-level device for diagnostics or advanced features.
 * @brief 获取底层设备，用于诊断或高级功能。
 * @param radio Application-level radio object. / 应用层无线模块对象。
 * @return Low-level device pointer, or NULL when not initialized.
 *         / 底层设备指针；未初始化时返回 NULL。
 */
Nrf24_t *Nrf24SimpleGetDevice(Nrf24Simple_t *radio);
