/**
 * @file    nrf24l01_test.h
 * @brief   nRF24L01+ MaixCAM communication test interface
 */
#pragma once

#include <stdint.h>

#define NRF24_COMM_TEST_MAGIC          (0x4E524632UL)
#define NRF24_COMM_TEST_CHANNEL        (76U)
#define NRF24_COMM_TEST_PAYLOAD_SIZE   (32U)

/**
 * @brief Communication test details kept in RAM for J-Link inspection.
 * @note  Every field is 32-bit so `mem32` output maps one-to-one to fields.
 */
typedef struct
{
    uint32_t magic;
    uint32_t initialized;
    uint32_t listening;
    uint32_t rx_packets;
    uint32_t valid_ping_packets;
    uint32_t pong_packets;
    uint32_t tx_failures;
    uint32_t last_sequence;
    uint32_t init_result;
    uint32_t listen_result;
    uint32_t last_rx_result;
    uint32_t last_tx_result;
    uint32_t last_status_result;
    uint32_t last_status;
    uint8_t last_payload[NRF24_COMM_TEST_PAYLOAD_SIZE];
} Nrf24CommunicationTestResult_t;

extern volatile Nrf24CommunicationTestResult_t g_nrf24_comm_result;

void Nrf24Stage1TestRun(void);
