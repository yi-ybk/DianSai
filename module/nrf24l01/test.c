#include "nrf24l01.h"
#include "nrf24l01_test.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static const uint8_t kRadioAddress[NRF24_MAX_ADDRESS_WIDTH] = {
    0xD2U, 0xF0U, 0xF0U, 0xF0U, 0xA5U
};
static const uint8_t kPingMagic[8] = {
    'M', 'C', 'A', 'M', 'P', 'I', 'N', 'G'
};
static const uint8_t kPongMagic[8] = {
    'M', '0', '_', '_', 'P', 'O', 'N', 'G'
};

static Nrf24_t s_radio;

volatile Nrf24CommunicationTestResult_t g_nrf24_comm_result;

static uint32_t Nrf24ReadSequence(const uint8_t *payload)
{
    return ((uint32_t)payload[8]) |
           ((uint32_t)payload[9] << 8U) |
           ((uint32_t)payload[10] << 16U) |
           ((uint32_t)payload[11] << 24U);
}

static void Nrf24BuildPong(uint8_t *payload, uint32_t sequence)
{
    memset(payload, 0, NRF24_COMM_TEST_PAYLOAD_SIZE);
    memcpy(payload, kPongMagic, sizeof(kPongMagic));
    payload[8]  = (uint8_t)sequence;
    payload[9]  = (uint8_t)(sequence >> 8U);
    payload[10] = (uint8_t)(sequence >> 16U);
    payload[11] = (uint8_t)(sequence >> 24U);
}

static void Nrf24SavePayload(const uint8_t *payload)
{
    uint32_t index;

    for (index = 0U; index < NRF24_COMM_TEST_PAYLOAD_SIZE; index++)
        g_nrf24_comm_result.last_payload[index] = payload[index];
}

static void Nrf24SaveReceiverDiagnostics(void)
{
    static const uint8_t registers[] = {
        NRF24_REG_CONFIG,
        NRF24_REG_EN_AA,
        NRF24_REG_EN_RXADDR,
        NRF24_REG_SETUP_AW,
        NRF24_REG_SETUP_RETR,
        NRF24_REG_RF_CH,
        NRF24_REG_RF_SETUP,
        NRF24_REG_STATUS,
        NRF24_REG_RX_PW_P0,
        NRF24_REG_FIFO_STATUS,
        NRF24_REG_DYNPD,
        NRF24_REG_FEATURE,
    };
    uint32_t index;
    uint8_t value;
    uint8_t address[NRF24_MAX_ADDRESS_WIDTH];

    for (index = 0U; index < sizeof(registers); index++)
    {
        value = 0xEEU;
        (void)Nrf24ReadRegister(&s_radio, registers[index], &value);
        g_nrf24_comm_result.last_payload[index] = value;
    }

    g_nrf24_comm_result.last_payload[12] =
        ((GPIO_NRF24_PORT->DOUT31_0 & GPIO_NRF24_CE_PIN) != 0U) ?
        1U :
        0U;
    g_nrf24_comm_result.last_payload[13] =
        Nrf24IrqIsActive(&s_radio) ? 1U : 0U;
    g_nrf24_comm_result.last_payload[14] = 0U;

    memset(address, 0xEE, sizeof(address));
    (void)Nrf24ReadRegisterBuffer(
        &s_radio,
        NRF24_REG_RX_ADDR_P0,
        address,
        sizeof(address));
    for (index = 0U; index < sizeof(address); index++)
        g_nrf24_comm_result.last_payload[15U + index] = address[index];

    memset(address, 0xEE, sizeof(address));
    (void)Nrf24ReadRegisterBuffer(
        &s_radio,
        NRF24_REG_TX_ADDR,
        address,
        sizeof(address));
    for (index = 0U; index < sizeof(address); index++)
        g_nrf24_comm_result.last_payload[20U + index] = address[index];
}

void Nrf24Stage1TestRun(void)
{
    Nrf24InitConfig_t config;
    Nrf24Result_t result;
    uint8_t payload[NRF24_COMM_TEST_PAYLOAD_SIZE];
    uint8_t response[NRF24_COMM_TEST_PAYLOAD_SIZE];
    uint8_t length;
    uint8_t pipe;
    uint32_t sequence;

    memset((void *)&g_nrf24_comm_result, 0, sizeof(g_nrf24_comm_result));
    g_nrf24_comm_result.magic = NRF24_COMM_TEST_MAGIC;
    Nrf24GetDefaultConfig(&config);

    config.port.spi      = SPI_NRF24_INST;
    config.port.software_spi = true;
    config.port.sck_port = GPIO_SPI_NRF24_SCLK_PORT;
    config.port.sck_pin = GPIO_SPI_NRF24_SCLK_PIN;
    config.port.sck_iomux = GPIO_SPI_NRF24_IOMUX_SCLK;
    config.port.mosi_port = GPIO_SPI_NRF24_PICO_PORT;
    config.port.mosi_pin = GPIO_SPI_NRF24_PICO_PIN;
    config.port.mosi_iomux = GPIO_SPI_NRF24_IOMUX_PICO;
    config.port.miso_port = GPIO_SPI_NRF24_POCI_PORT;
    config.port.miso_pin = GPIO_SPI_NRF24_POCI_PIN;
    config.port.miso_iomux = GPIO_SPI_NRF24_IOMUX_POCI;
    config.port.ce_port  = GPIO_NRF24_PORT;
    config.port.ce_pin   = GPIO_NRF24_CE_PIN;
    config.port.csn_port = GPIO_NRF24_PORT;
    config.port.csn_pin  = GPIO_NRF24_CSN_PIN;
    config.port.irq_port = GPIO_NRF24_PORT;
    config.port.irq_pin  = GPIO_NRF24_IRQ_PIN;

    config.channel             = NRF24_COMM_TEST_CHANNEL;
    config.data_rate           = NRF24_DATA_RATE_1_MBPS;
    config.output_power        = NRF24_OUTPUT_POWER_NEG_18_DBM;
    config.payload_size        = NRF24_COMM_TEST_PAYLOAD_SIZE;
    config.retransmit_delay_us = 1000U;
    config.retransmit_count    = 15U;
    config.auto_ack            = true;
    config.dynamic_payloads    = false;
    memcpy(config.tx_address, kRadioAddress, sizeof(kRadioAddress));
    memcpy(config.rx_address_p0, kRadioAddress, sizeof(kRadioAddress));

    result = Nrf24Init(&s_radio, &config);
    g_nrf24_comm_result.init_result = (uint32_t)result;
    if (result != NRF24_RESULT_OK)
    {
        for (;;)
            __NOP();
    }
    g_nrf24_comm_result.initialized = 1U;

    result = Nrf24StartListening(&s_radio);
    g_nrf24_comm_result.listen_result = (uint32_t)result;
    if (result != NRF24_RESULT_OK)
    {
        for (;;)
            __NOP();
    }
    g_nrf24_comm_result.listening = 1U;
    Nrf24SaveReceiverDiagnostics();

    for (;;)
    {
        if (!Nrf24DataAvailable(&s_radio, &pipe))
        {
            uint8_t rpd = 0U;
            if ((Nrf24ReadRegister(&s_radio, NRF24_REG_RPD, &rpd) ==
                 NRF24_RESULT_OK) &&
                ((rpd & 0x01U) != 0U))
            {
                g_nrf24_comm_result.last_payload[14] = 1U;
            }
            Nrf24PortDelayMs(1U);
            continue;
        }

        result = Nrf24Receive(
            &s_radio, payload, sizeof(payload), &length, &pipe);
        g_nrf24_comm_result.last_rx_result = (uint32_t)result;
        if (result != NRF24_RESULT_OK)
            continue;

        g_nrf24_comm_result.rx_packets++;
        Nrf24SavePayload(payload);
        if ((length != NRF24_COMM_TEST_PAYLOAD_SIZE) ||
            (memcmp(payload, kPingMagic, sizeof(kPingMagic)) != 0))
        {
            continue;
        }

        sequence = Nrf24ReadSequence(payload);
        g_nrf24_comm_result.last_sequence = sequence;
        g_nrf24_comm_result.valid_ping_packets++;

        Nrf24BuildPong(response, sequence);
        Nrf24PortDelayMs(5U);
        g_nrf24_comm_result.listening = 0U;
        result = Nrf24Transmit(
            &s_radio, response, sizeof(response), 100U);
        g_nrf24_comm_result.last_tx_result = (uint32_t)result;
        if (result == NRF24_RESULT_OK)
            g_nrf24_comm_result.pong_packets++;
        else
            g_nrf24_comm_result.tx_failures++;

        result = Nrf24StartListening(&s_radio);
        g_nrf24_comm_result.listen_result = (uint32_t)result;
        g_nrf24_comm_result.listening =
            (result == NRF24_RESULT_OK) ? 1U : 0U;

        g_nrf24_comm_result.last_status_result =
            (uint32_t)Nrf24GetStatus(
                &s_radio, (uint8_t *)&g_nrf24_comm_result.last_status);
    }
}
