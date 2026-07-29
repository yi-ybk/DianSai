/**
 * @file    nrf24l01.h
 * @brief   Platform-aware nRF24L01+ driver interface for MSPM0G3507
 */
#pragma once

#include "nrf24l01_port_mspm0.h"
#include "nrf24l01_regs.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    NRF24_RESULT_OK = 0,
    NRF24_RESULT_INVALID_ARGUMENT,
    NRF24_RESULT_NOT_INITIALIZED,
    NRF24_RESULT_IO_ERROR,
    NRF24_RESULT_DEVICE_NOT_FOUND,
    NRF24_RESULT_NO_DATA,
    NRF24_RESULT_BUFFER_TOO_SMALL,
    NRF24_RESULT_INVALID_PAYLOAD_WIDTH,
    NRF24_RESULT_MAX_RETRIES,
    NRF24_RESULT_TIMEOUT,
    NRF24_RESULT_RTOS_NOT_RUNNING,
    NRF24_RESULT_LOCK_TIMEOUT,
} Nrf24Result_t;

typedef enum
{
    NRF24_DATA_RATE_250_KBPS = 0,
    NRF24_DATA_RATE_1_MBPS,
    NRF24_DATA_RATE_2_MBPS,
} Nrf24DataRate_t;

typedef enum
{
    NRF24_OUTPUT_POWER_NEG_18_DBM = 0,
    NRF24_OUTPUT_POWER_NEG_12_DBM,
    NRF24_OUTPUT_POWER_NEG_6_DBM,
    NRF24_OUTPUT_POWER_0_DBM,
} Nrf24OutputPower_t;

typedef enum
{
    NRF24_CRC_1_BYTE = 1,
    NRF24_CRC_2_BYTES = 2,
} Nrf24CrcLength_t;

/** @brief nRF24L01+ initialization and radio parameters. */
typedef struct
{
    Nrf24PortConfig_t port;
    uint8_t channel;
    Nrf24DataRate_t data_rate;
    Nrf24OutputPower_t output_power;
    Nrf24CrcLength_t crc_length;
    uint8_t address_width;
    uint8_t payload_size;
    uint16_t retransmit_delay_us;
    uint8_t retransmit_count;
    bool auto_ack;
    bool dynamic_payloads;
    uint8_t tx_address[NRF24_MAX_ADDRESS_WIDTH];
    uint8_t rx_address_p0[NRF24_MAX_ADDRESS_WIDTH];
} Nrf24InitConfig_t;

/** @brief One nRF24L01+ device instance. */
typedef struct
{
    Nrf24Port_t port;
    Nrf24InitConfig_t config;
    uint8_t last_status;
    bool initialized;
} Nrf24_t;

void Nrf24GetDefaultConfig(Nrf24InitConfig_t *config);
Nrf24Result_t Nrf24Init(Nrf24_t *device, const Nrf24InitConfig_t *config);
bool Nrf24IsInitialized(const Nrf24_t *device);
bool Nrf24CheckConnection(Nrf24_t *device);

Nrf24Result_t Nrf24ReadRegister(Nrf24_t *device,
                                uint8_t reg,
                                uint8_t *value);
Nrf24Result_t Nrf24ReadRegisterBuffer(Nrf24_t *device,
                                      uint8_t reg,
                                      uint8_t *data,
                                      uint8_t length);
Nrf24Result_t Nrf24WriteRegister(Nrf24_t *device,
                                 uint8_t reg,
                                 uint8_t value);
Nrf24Result_t Nrf24GetStatus(Nrf24_t *device, uint8_t *status);
Nrf24Result_t Nrf24ClearIrq(Nrf24_t *device, uint8_t irq_mask);
bool Nrf24IrqIsActive(const Nrf24_t *device);

Nrf24Result_t Nrf24SetChannel(Nrf24_t *device, uint8_t channel);
Nrf24Result_t Nrf24SetDataRate(Nrf24_t *device,
                               Nrf24DataRate_t data_rate);
Nrf24Result_t Nrf24SetOutputPower(Nrf24_t *device,
                                  Nrf24OutputPower_t output_power);
Nrf24Result_t Nrf24SetTxAddress(Nrf24_t *device,
                                const uint8_t *address);
Nrf24Result_t Nrf24SetRxPipe0Address(Nrf24_t *device,
                                     const uint8_t *address);

Nrf24Result_t Nrf24PowerDown(Nrf24_t *device);
Nrf24Result_t Nrf24StopListening(Nrf24_t *device);
Nrf24Result_t Nrf24StartListening(Nrf24_t *device);
Nrf24Result_t Nrf24FlushTx(Nrf24_t *device);
Nrf24Result_t Nrf24FlushRx(Nrf24_t *device);

Nrf24Result_t Nrf24Transmit(Nrf24_t *device,
                            const uint8_t *payload,
                            uint8_t length,
                            uint32_t timeout_ms);
bool Nrf24DataAvailable(Nrf24_t *device, uint8_t *pipe);
Nrf24Result_t Nrf24Receive(Nrf24_t *device,
                           uint8_t *payload,
                           uint8_t capacity,
                           uint8_t *length,
                           uint8_t *pipe);
