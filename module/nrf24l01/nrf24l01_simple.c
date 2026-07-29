/**
 * @file    nrf24l01_simple.c
 * @brief   Application-level nRF24L01+ convenience API for MSPM0G3507
 * @brief   MSPM0G3507 的 nRF24L01+ 应用层便捷 API 实现
 */
#include "nrf24l01_simple.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <string.h>

/* Verified common address for MaixCAM communication.
 * 与 MaixCAM 通信时已验证的公共地址。 */
static const uint8_t kTianmengxingAddress[NRF24_MAX_ADDRESS_WIDTH] = {
    0xD2U, 0xF0U, 0xF0U, 0xF0U, 0xA5U
};

void Nrf24SimpleGetTianmengxingConfig(Nrf24InitConfig_t *config)
{
    if (config == NULL)
        return;

    Nrf24GetDefaultConfig(config);

    /* Fill the Tianmengxing pin mapping verified by the communication test.
     * 填充已通过通信测试的天猛星引脚映射。 */
    config->port.spi        = SPI_NRF24_INST;
    config->port.software_spi = true;
    config->port.sck_port   = GPIO_SPI_NRF24_SCLK_PORT;
    config->port.sck_pin    = GPIO_SPI_NRF24_SCLK_PIN;
    config->port.sck_iomux  = GPIO_SPI_NRF24_IOMUX_SCLK;
    config->port.mosi_port  = GPIO_SPI_NRF24_PICO_PORT;
    config->port.mosi_pin   = GPIO_SPI_NRF24_PICO_PIN;
    config->port.mosi_iomux = GPIO_SPI_NRF24_IOMUX_PICO;
    config->port.miso_port  = GPIO_SPI_NRF24_POCI_PORT;
    config->port.miso_pin   = GPIO_SPI_NRF24_POCI_PIN;
    config->port.miso_iomux = GPIO_SPI_NRF24_IOMUX_POCI;
    config->port.ce_port    = GPIO_NRF24_PORT;
    config->port.ce_pin     = GPIO_NRF24_CE_PIN;
    config->port.csn_port   = GPIO_NRF24_PORT;
    config->port.csn_pin    = GPIO_NRF24_CSN_PIN;
    config->port.irq_port   = GPIO_NRF24_PORT;
    config->port.irq_pin    = GPIO_NRF24_IRQ_PIN;

    /* Keep both endpoints on the same verified radio parameters.
     * 保持两端使用相同且已经验证的无线参数。 */
    config->channel             = 76U;
    config->data_rate           = NRF24_DATA_RATE_1_MBPS;
    config->output_power        = NRF24_OUTPUT_POWER_NEG_18_DBM;
    config->crc_length          = NRF24_CRC_2_BYTES;
    config->address_width       = NRF24_MAX_ADDRESS_WIDTH;
    config->payload_size        = NRF24_MAX_PAYLOAD_SIZE;
    config->retransmit_delay_us = 1000U;
    config->retransmit_count    = 15U;
    config->auto_ack            = true;
    config->dynamic_payloads    = false;
    memcpy(config->tx_address,
           kTianmengxingAddress,
           sizeof(kTianmengxingAddress));
    memcpy(config->rx_address_p0,
           kTianmengxingAddress,
           sizeof(kTianmengxingAddress));
}

Nrf24Result_t Nrf24SimpleInit(Nrf24Simple_t *radio,
                              const Nrf24InitConfig_t *config,
                              bool start_receiving)
{
    Nrf24Result_t result;

    if ((radio == NULL) || (config == NULL))
        return NRF24_RESULT_INVALID_ARGUMENT;

    memset(radio, 0, sizeof(*radio));
    result = Nrf24Init(&radio->device, config);
    if (result != NRF24_RESULT_OK)
        return result;

    if (!start_receiving)
        return NRF24_RESULT_OK;

    result = Nrf24StartListening(&radio->device);
    if (result == NRF24_RESULT_OK)
        radio->receiving = true;

    return result;
}

bool Nrf24SimpleIsInitialized(const Nrf24Simple_t *radio)
{
    return (radio != NULL) && Nrf24IsInitialized(&radio->device);
}

Nrf24Result_t Nrf24SimpleSend(Nrf24Simple_t *radio,
                              const uint8_t *data,
                              uint8_t length,
                              uint32_t timeout_ms,
                              bool resume_receive)
{
    Nrf24Result_t result;
    Nrf24Result_t resume_result;
    uint8_t fixed_payload[NRF24_MAX_PAYLOAD_SIZE];
    const uint8_t *payload;
    uint8_t wire_length;
    bool was_receiving;

    if (!Nrf24SimpleIsInitialized(radio))
        return NRF24_RESULT_NOT_INITIALIZED;
    if ((data == NULL) || (length == 0U) ||
        (length > NRF24_MAX_PAYLOAD_SIZE))
    {
        return NRF24_RESULT_INVALID_ARGUMENT;
    }

    payload = data;
    wire_length = length;
    if (!radio->device.config.dynamic_payloads)
    {
        if (length > radio->device.config.payload_size)
            return NRF24_RESULT_INVALID_ARGUMENT;

        /* Pad short application data to the configured fixed wire length.
         * 将较短的应用数据补零到配置的固定空中负载长度。 */
        memset(fixed_payload, 0, sizeof(fixed_payload));
        memcpy(fixed_payload, data, length);
        payload = fixed_payload;
        wire_length = radio->device.config.payload_size;
    }

    /* Nrf24Transmit enters PTX mode; remember whether RX must be restored.
     * Nrf24Transmit 会进入 PTX 模式，因此先记录发送后是否需要恢复接收。 */
    was_receiving = radio->receiving;
    radio->receiving = false;
    result = Nrf24Transmit(
        &radio->device, payload, wire_length, timeout_ms);

    if (resume_receive && was_receiving)
    {
        resume_result = Nrf24StartListening(&radio->device);
        if (resume_result == NRF24_RESULT_OK)
            radio->receiving = true;
        if (result == NRF24_RESULT_OK)
            result = resume_result;
    }

    return result;
}

Nrf24Result_t Nrf24SimpleStartReceive(Nrf24Simple_t *radio,
                                      bool clear_pending)
{
    Nrf24Result_t result;

    if (!Nrf24SimpleIsInitialized(radio))
        return NRF24_RESULT_NOT_INITIALIZED;

    if (clear_pending)
    {
        /* Discard pending packets only when explicitly requested.
         * 仅在调用者明确要求时丢弃尚未读取的数据包。 */
        result = Nrf24FlushRx(&radio->device);
        if (result != NRF24_RESULT_OK)
            return result;
    }

    if (radio->receiving)
        return NRF24_RESULT_OK;

    result = Nrf24StartListening(&radio->device);
    if (result == NRF24_RESULT_OK)
        radio->receiving = true;

    return result;
}

Nrf24Result_t Nrf24SimpleStopReceive(Nrf24Simple_t *radio)
{
    Nrf24Result_t result;

    if (!Nrf24SimpleIsInitialized(radio))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (!radio->receiving)
        return NRF24_RESULT_OK;

    result = Nrf24StopListening(&radio->device);
    if (result == NRF24_RESULT_OK)
        radio->receiving = false;

    return result;
}

Nrf24Result_t Nrf24SimpleReceive(Nrf24Simple_t *radio,
                                 uint8_t *data,
                                 uint8_t capacity,
                                 uint8_t *length,
                                 uint8_t *pipe,
                                 uint32_t timeout_ms)
{
    Nrf24Result_t result;
    uint32_t elapsed_ms = 0U;

    if (!Nrf24SimpleIsInitialized(radio))
        return NRF24_RESULT_NOT_INITIALIZED;
    if ((data == NULL) || (capacity == 0U) || (length == NULL))
        return NRF24_RESULT_INVALID_ARGUMENT;

    result = Nrf24SimpleStartReceive(radio, false);
    if (result != NRF24_RESULT_OK)
        return result;

    for (;;)
    {
        /* Poll once per millisecond so timeout_ms keeps millisecond semantics.
         * 每毫秒轮询一次，使 timeout_ms 保持明确的毫秒语义。 */
        result = Nrf24Receive(
            &radio->device, data, capacity, length, pipe);
        if (result != NRF24_RESULT_NO_DATA)
            return result;

        if (timeout_ms == 0U)
            return NRF24_RESULT_NO_DATA;
        if (elapsed_ms >= timeout_ms)
            return NRF24_RESULT_TIMEOUT;

        Nrf24PortDelayMs(1U);
        elapsed_ms++;
    }
}

bool Nrf24SimpleDataAvailable(Nrf24Simple_t *radio)
{
    if (!Nrf24SimpleIsInitialized(radio))
        return false;

    return Nrf24DataAvailable(&radio->device, NULL);
}

Nrf24Result_t Nrf24SimplePowerDown(Nrf24Simple_t *radio)
{
    Nrf24Result_t result;

    if (!Nrf24SimpleIsInitialized(radio))
        return NRF24_RESULT_NOT_INITIALIZED;

    result = Nrf24PowerDown(&radio->device);
    if (result == NRF24_RESULT_OK)
        radio->receiving = false;

    return result;
}

Nrf24_t *Nrf24SimpleGetDevice(Nrf24Simple_t *radio)
{
    if (!Nrf24SimpleIsInitialized(radio))
        return NULL;

    return &radio->device;
}
