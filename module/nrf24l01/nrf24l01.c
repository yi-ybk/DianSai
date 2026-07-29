/**
 * @file    nrf24l01.c
 * @brief   nRF24L01+ driver implementation for the MSPM0 port layer
 */
#include "nrf24l01.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define NRF24_POWER_ON_DELAY_MS       100U
#define NRF24_POWER_UP_DELAY_US       5000U
#define NRF24_MODE_SETTLE_DELAY_US    150U
#define NRF24_CE_PULSE_US             15U
#define NRF24_TX_POLL_INTERVAL_US     50U

static bool Nrf24ConfigIsValid(const Nrf24InitConfig_t *config);
static bool Nrf24IsReady(const Nrf24_t *device);
static Nrf24Result_t Nrf24ReadRegisterInternal(Nrf24_t *device,
                                               uint8_t reg,
                                               uint8_t *value);
static Nrf24Result_t Nrf24WriteRegisterInternal(Nrf24_t *device,
                                                uint8_t reg,
                                                uint8_t value);
static Nrf24Result_t Nrf24ReadBufferInternal(Nrf24_t *device,
                                             uint8_t command,
                                             uint8_t *data,
                                             uint8_t length);
static Nrf24Result_t Nrf24WriteBufferInternal(Nrf24_t *device,
                                              uint8_t command,
                                              const uint8_t *data,
                                              uint8_t length);
static Nrf24Result_t Nrf24CommandInternal(Nrf24_t *device, uint8_t command);
static Nrf24Result_t Nrf24CommandWithDataInternal(Nrf24_t *device,
                                                  uint8_t command,
                                                  uint8_t data);
static Nrf24Result_t Nrf24ReadStatusInternal(Nrf24_t *device, uint8_t *status);
static Nrf24Result_t Nrf24ConfigureFeatureRegisters(Nrf24_t *device);
static Nrf24Result_t Nrf24ApplyRadioConfig(Nrf24_t *device);
static uint8_t Nrf24BuildRfSetup(Nrf24DataRate_t data_rate,
                                 Nrf24OutputPower_t output_power);
static uint8_t Nrf24BuildSetupRetr(const Nrf24InitConfig_t *config);
static uint8_t Nrf24AddressWidthToRegister(uint8_t address_width);
static uint32_t Nrf24TimeoutMsToUs(uint32_t timeout_ms);

void Nrf24GetDefaultConfig(Nrf24InitConfig_t *config)
{
    static const uint8_t default_address[NRF24_MAX_ADDRESS_WIDTH] = {
        0xE7U, 0xE7U, 0xE7U, 0xE7U, 0xE7U
    };

    if (config == NULL)
        return;

    memset(config, 0, sizeof(*config));
    config->channel             = 76U;
    config->data_rate           = NRF24_DATA_RATE_1_MBPS;
    config->output_power        = NRF24_OUTPUT_POWER_0_DBM;
    config->crc_length          = NRF24_CRC_2_BYTES;
    config->address_width       = NRF24_MAX_ADDRESS_WIDTH;
    config->payload_size        = NRF24_MAX_PAYLOAD_SIZE;
    config->retransmit_delay_us = 500U;
    config->retransmit_count    = 5U;
    config->auto_ack            = true;
    config->dynamic_payloads    = true;
    memcpy(config->tx_address, default_address, sizeof(default_address));
    memcpy(config->rx_address_p0, default_address, sizeof(default_address));
}

Nrf24Result_t Nrf24Init(Nrf24_t *device, const Nrf24InitConfig_t *config)
{
    Nrf24Result_t result;
    uint8_t setup_aw;
    uint8_t expected_setup_aw;

    if ((device == NULL) || (!Nrf24ConfigIsValid(config)))
        return NRF24_RESULT_INVALID_ARGUMENT;

    memset(device, 0, sizeof(*device));
    device->config = *config;

    if (!Nrf24PortInit(&device->port, &device->config.port))
        return NRF24_RESULT_IO_ERROR;

    Nrf24PortSetCe(&device->port, false);
    Nrf24PortSetCsn(&device->port, true);
    Nrf24PortDelayMs(NRF24_POWER_ON_DELAY_MS);

    expected_setup_aw = Nrf24AddressWidthToRegister(device->config.address_width);
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_SETUP_AW,
                                        expected_setup_aw);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24ReadRegisterInternal(device, NRF24_REG_SETUP_AW, &setup_aw);
    if (result != NRF24_RESULT_OK)
        return result;
    if ((setup_aw & 0x03U) != expected_setup_aw)
        return NRF24_RESULT_DEVICE_NOT_FOUND;

    result = Nrf24ApplyRadioConfig(device);
    if (result != NRF24_RESULT_OK)
        return result;

    device->initialized = true;
    return NRF24_RESULT_OK;
}

bool Nrf24IsInitialized(const Nrf24_t *device)
{
    return Nrf24IsReady(device);
}

bool Nrf24CheckConnection(Nrf24_t *device)
{
    uint8_t setup_aw;
    uint8_t expected_setup_aw;

    if (!Nrf24IsReady(device))
        return false;

    expected_setup_aw = Nrf24AddressWidthToRegister(device->config.address_width);
    if (Nrf24ReadRegisterInternal(device, NRF24_REG_SETUP_AW, &setup_aw) !=
        NRF24_RESULT_OK)
    {
        return false;
    }

    return (setup_aw & 0x03U) == expected_setup_aw;
}

Nrf24Result_t Nrf24ReadRegister(Nrf24_t *device,
                                uint8_t reg,
                                uint8_t *value)
{
    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if ((value == NULL) || (reg > NRF24_REG_FEATURE))
        return NRF24_RESULT_INVALID_ARGUMENT;

    return Nrf24ReadRegisterInternal(device, reg, value);
}

Nrf24Result_t Nrf24ReadRegisterBuffer(Nrf24_t *device,
                                      uint8_t reg,
                                      uint8_t *data,
                                      uint8_t length)
{
    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if ((data == NULL) || (length == 0U) ||
        (reg > NRF24_REG_FEATURE))
    {
        return NRF24_RESULT_INVALID_ARGUMENT;
    }

    return Nrf24ReadBufferInternal(
        device,
        (uint8_t)(NRF24_CMD_R_REGISTER | (reg & NRF24_REGISTER_MASK)),
        data,
        length);
}

Nrf24Result_t Nrf24WriteRegister(Nrf24_t *device,
                                 uint8_t reg,
                                 uint8_t value)
{
    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (reg > NRF24_REG_FEATURE)
        return NRF24_RESULT_INVALID_ARGUMENT;

    return Nrf24WriteRegisterInternal(device, reg, value);
}

Nrf24Result_t Nrf24GetStatus(Nrf24_t *device, uint8_t *status)
{
    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (status == NULL)
        return NRF24_RESULT_INVALID_ARGUMENT;

    return Nrf24ReadStatusInternal(device, status);
}

Nrf24Result_t Nrf24ClearIrq(Nrf24_t *device, uint8_t irq_mask)
{
    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;

    irq_mask &= NRF24_STATUS_IRQ_MASK;
    if (irq_mask == 0U)
        return NRF24_RESULT_INVALID_ARGUMENT;

    return Nrf24WriteRegisterInternal(device, NRF24_REG_STATUS, irq_mask);
}

bool Nrf24IrqIsActive(const Nrf24_t *device)
{
    if (!Nrf24IsReady(device))
        return false;

    return Nrf24PortIrqIsActive(&device->port);
}

Nrf24Result_t Nrf24SetChannel(Nrf24_t *device, uint8_t channel)
{
    Nrf24Result_t result;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (channel > NRF24_MAX_RF_CHANNEL)
        return NRF24_RESULT_INVALID_ARGUMENT;

    result = Nrf24WriteRegisterInternal(device, NRF24_REG_RF_CH, channel);
    if (result == NRF24_RESULT_OK)
        device->config.channel = channel;

    return result;
}

Nrf24Result_t Nrf24SetDataRate(Nrf24_t *device,
                               Nrf24DataRate_t data_rate)
{
    Nrf24Result_t result;
    uint8_t rf_setup;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (data_rate > NRF24_DATA_RATE_2_MBPS)
        return NRF24_RESULT_INVALID_ARGUMENT;
    if ((data_rate == NRF24_DATA_RATE_250_KBPS) &&
        (device->config.retransmit_delay_us < 500U))
    {
        return NRF24_RESULT_INVALID_ARGUMENT;
    }

    result = Nrf24ReadRegisterInternal(device, NRF24_REG_RF_SETUP, &rf_setup);
    if (result != NRF24_RESULT_OK)
        return result;

    rf_setup &= (uint8_t)~(NRF24_RF_SETUP_RF_DR_LOW |
                           NRF24_RF_SETUP_RF_DR_HIGH);
    if (data_rate == NRF24_DATA_RATE_250_KBPS)
        rf_setup |= NRF24_RF_SETUP_RF_DR_LOW;
    else if (data_rate == NRF24_DATA_RATE_2_MBPS)
        rf_setup |= NRF24_RF_SETUP_RF_DR_HIGH;

    result = Nrf24WriteRegisterInternal(device, NRF24_REG_RF_SETUP, rf_setup);
    if (result == NRF24_RESULT_OK)
        device->config.data_rate = data_rate;

    return result;
}

Nrf24Result_t Nrf24SetOutputPower(Nrf24_t *device,
                                  Nrf24OutputPower_t output_power)
{
    Nrf24Result_t result;
    uint8_t rf_setup;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (output_power > NRF24_OUTPUT_POWER_0_DBM)
        return NRF24_RESULT_INVALID_ARGUMENT;

    result = Nrf24ReadRegisterInternal(device, NRF24_REG_RF_SETUP, &rf_setup);
    if (result != NRF24_RESULT_OK)
        return result;

    rf_setup &= (uint8_t)~NRF24_RF_SETUP_RF_PWR_MASK;
    rf_setup |= (uint8_t)((uint8_t)output_power << 1U);
    result = Nrf24WriteRegisterInternal(device, NRF24_REG_RF_SETUP, rf_setup);
    if (result == NRF24_RESULT_OK)
        device->config.output_power = output_power;

    return result;
}

Nrf24Result_t Nrf24SetTxAddress(Nrf24_t *device,
                                const uint8_t *address)
{
    Nrf24Result_t result;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (address == NULL)
        return NRF24_RESULT_INVALID_ARGUMENT;

    result = Nrf24WriteBufferInternal(
        device,
        (uint8_t)(NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR),
        address,
        device->config.address_width);
    if (result == NRF24_RESULT_OK)
    {
        memcpy(device->config.tx_address,
               address,
               device->config.address_width);
    }

    return result;
}

Nrf24Result_t Nrf24SetRxPipe0Address(Nrf24_t *device,
                                     const uint8_t *address)
{
    Nrf24Result_t result;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if (address == NULL)
        return NRF24_RESULT_INVALID_ARGUMENT;

    result = Nrf24WriteBufferInternal(
        device,
        (uint8_t)(NRF24_CMD_W_REGISTER | NRF24_REG_RX_ADDR_P0),
        address,
        device->config.address_width);
    if (result == NRF24_RESULT_OK)
    {
        memcpy(device->config.rx_address_p0,
               address,
               device->config.address_width);
    }

    return result;
}

Nrf24Result_t Nrf24PowerDown(Nrf24_t *device)
{
    Nrf24Result_t result;
    uint8_t config;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;

    Nrf24PortSetCe(&device->port, false);
    result = Nrf24ReadRegisterInternal(device, NRF24_REG_CONFIG, &config);
    if (result != NRF24_RESULT_OK)
        return result;

    config &= (uint8_t)~NRF24_CONFIG_PWR_UP;
    return Nrf24WriteRegisterInternal(device, NRF24_REG_CONFIG, config);
}

Nrf24Result_t Nrf24StopListening(Nrf24_t *device)
{
    Nrf24Result_t result;
    uint8_t config;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;

    Nrf24PortSetCe(&device->port, false);
    result = Nrf24ReadRegisterInternal(device, NRF24_REG_CONFIG, &config);
    if (result != NRF24_RESULT_OK)
        return result;

    config |= NRF24_CONFIG_PWR_UP;
    config &= (uint8_t)~NRF24_CONFIG_PRIM_RX;
    result = Nrf24WriteRegisterInternal(device, NRF24_REG_CONFIG, config);
    if (result == NRF24_RESULT_OK)
        Nrf24PortDelayUs(NRF24_MODE_SETTLE_DELAY_US);

    return result;
}

Nrf24Result_t Nrf24StartListening(Nrf24_t *device)
{
    Nrf24Result_t result;
    uint8_t config;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;

    Nrf24PortSetCe(&device->port, false);
    result = Nrf24ReadRegisterInternal(device, NRF24_REG_CONFIG, &config);
    if (result != NRF24_RESULT_OK)
        return result;

    config |= NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX;
    result = Nrf24WriteRegisterInternal(device, NRF24_REG_CONFIG, config);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_STATUS,
                                        NRF24_STATUS_RX_DR);
    if (result != NRF24_RESULT_OK)
        return result;

    Nrf24PortSetCe(&device->port, true);
    Nrf24PortDelayUs(NRF24_MODE_SETTLE_DELAY_US);
    return NRF24_RESULT_OK;
}

Nrf24Result_t Nrf24FlushTx(Nrf24_t *device)
{
    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;

    return Nrf24CommandInternal(device, NRF24_CMD_FLUSH_TX);
}

Nrf24Result_t Nrf24FlushRx(Nrf24_t *device)
{
    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;

    return Nrf24CommandInternal(device, NRF24_CMD_FLUSH_RX);
}

Nrf24Result_t Nrf24Transmit(Nrf24_t *device,
                            const uint8_t *payload,
                            uint8_t length,
                            uint32_t timeout_ms)
{
    Nrf24Result_t result;
    uint32_t elapsed_us = 0U;
    uint32_t timeout_us;
    uint8_t status;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if ((payload == NULL) || (length == 0U) ||
        (length > NRF24_MAX_PAYLOAD_SIZE))
    {
        return NRF24_RESULT_INVALID_ARGUMENT;
    }
    if ((!device->config.dynamic_payloads) &&
        (length != device->config.payload_size))
    {
        return NRF24_RESULT_INVALID_ARGUMENT;
    }

    result = Nrf24StopListening(device);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24CommandInternal(device, NRF24_CMD_FLUSH_TX);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_STATUS,
                                        NRF24_STATUS_TX_DS |
                                        NRF24_STATUS_MAX_RT);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteBufferInternal(device,
                                      NRF24_CMD_W_TX_PAYLOAD,
                                      payload,
                                      length);
    if (result != NRF24_RESULT_OK)
        return result;

    Nrf24PortSetCe(&device->port, true);
    Nrf24PortDelayUs(NRF24_CE_PULSE_US);
    Nrf24PortSetCe(&device->port, false);

    timeout_us = Nrf24TimeoutMsToUs(timeout_ms);
    for (;;)
    {
        result = Nrf24ReadStatusInternal(device, &status);
        if (result != NRF24_RESULT_OK)
            return result;

        if ((status & NRF24_STATUS_TX_DS) != 0U)
        {
            (void)Nrf24WriteRegisterInternal(device,
                                             NRF24_REG_STATUS,
                                             NRF24_STATUS_TX_DS);
            return NRF24_RESULT_OK;
        }

        if ((status & NRF24_STATUS_MAX_RT) != 0U)
        {
            (void)Nrf24WriteRegisterInternal(device,
                                             NRF24_REG_STATUS,
                                             NRF24_STATUS_MAX_RT);
            (void)Nrf24CommandInternal(device, NRF24_CMD_FLUSH_TX);
            return NRF24_RESULT_MAX_RETRIES;
        }

        if (elapsed_us >= timeout_us)
        {
            (void)Nrf24CommandInternal(device, NRF24_CMD_FLUSH_TX);
            return NRF24_RESULT_TIMEOUT;
        }

        Nrf24PortDelayUs(NRF24_TX_POLL_INTERVAL_US);
        if ((timeout_us - elapsed_us) < NRF24_TX_POLL_INTERVAL_US)
            elapsed_us = timeout_us;
        else
            elapsed_us += NRF24_TX_POLL_INTERVAL_US;
    }
}

bool Nrf24DataAvailable(Nrf24_t *device, uint8_t *pipe)
{
    uint8_t fifo_status;
    uint8_t status;
    uint8_t pipe_number;

    if (!Nrf24IsReady(device))
        return false;
    if (Nrf24ReadRegisterInternal(device,
                                  NRF24_REG_FIFO_STATUS,
                                  &fifo_status) != NRF24_RESULT_OK)
    {
        return false;
    }
    if ((fifo_status & NRF24_FIFO_STATUS_RX_EMPTY) != 0U)
        return false;
    if (Nrf24ReadStatusInternal(device, &status) != NRF24_RESULT_OK)
        return false;

    pipe_number = (uint8_t)((status & NRF24_STATUS_RX_P_NO_MASK) >>
                            NRF24_STATUS_RX_P_NO_SHIFT);
    if (pipe != NULL)
        *pipe = (pipe_number <= 5U) ? pipe_number : 0xFFU;

    return true;
}

Nrf24Result_t Nrf24Receive(Nrf24_t *device,
                           uint8_t *payload,
                           uint8_t capacity,
                           uint8_t *length,
                           uint8_t *pipe)
{
    Nrf24Result_t result;
    uint8_t fifo_status;
    uint8_t payload_width;
    uint8_t status;
    uint8_t pipe_number;

    if (!Nrf24IsReady(device))
        return NRF24_RESULT_NOT_INITIALIZED;
    if ((payload == NULL) || (capacity == 0U) || (length == NULL))
        return NRF24_RESULT_INVALID_ARGUMENT;

    *length = 0U;
    result = Nrf24ReadRegisterInternal(device,
                                       NRF24_REG_FIFO_STATUS,
                                       &fifo_status);
    if (result != NRF24_RESULT_OK)
        return result;
    if ((fifo_status & NRF24_FIFO_STATUS_RX_EMPTY) != 0U)
        return NRF24_RESULT_NO_DATA;

    result = Nrf24ReadStatusInternal(device, &status);
    if (result != NRF24_RESULT_OK)
        return result;
    pipe_number = (uint8_t)((status & NRF24_STATUS_RX_P_NO_MASK) >>
                            NRF24_STATUS_RX_P_NO_SHIFT);
    if (pipe != NULL)
        *pipe = (pipe_number <= 5U) ? pipe_number : 0xFFU;

    if (device->config.dynamic_payloads)
    {
        result = Nrf24ReadBufferInternal(device,
                                         NRF24_CMD_R_RX_PL_WID,
                                         &payload_width,
                                         1U);
        if (result != NRF24_RESULT_OK)
            return result;
    }
    else
    {
        payload_width = device->config.payload_size;
    }

    if ((payload_width == 0U) || (payload_width > NRF24_MAX_PAYLOAD_SIZE))
    {
        (void)Nrf24CommandInternal(device, NRF24_CMD_FLUSH_RX);
        (void)Nrf24WriteRegisterInternal(device,
                                         NRF24_REG_STATUS,
                                         NRF24_STATUS_RX_DR);
        return NRF24_RESULT_INVALID_PAYLOAD_WIDTH;
    }

    *length = payload_width;
    if (capacity < payload_width)
        return NRF24_RESULT_BUFFER_TOO_SMALL;

    result = Nrf24ReadBufferInternal(device,
                                     NRF24_CMD_R_RX_PAYLOAD,
                                     payload,
                                     payload_width);
    if (result != NRF24_RESULT_OK)
        return result;

    return Nrf24WriteRegisterInternal(device,
                                      NRF24_REG_STATUS,
                                      NRF24_STATUS_RX_DR);
}

static bool Nrf24ConfigIsValid(const Nrf24InitConfig_t *config)
{
    bool irq_disabled;
    bool irq_enabled;

    if ((config == NULL) ||
        (config->port.spi == NULL) ||
        (config->port.ce_port == NULL) ||
        (config->port.ce_pin == 0U) ||
        (config->port.csn_port == NULL) ||
        (config->port.csn_pin == 0U))
    {
        return false;
    }

    irq_disabled = (config->port.irq_port == NULL) &&
                   (config->port.irq_pin == 0U);
    irq_enabled  = (config->port.irq_port != NULL) &&
                   (config->port.irq_pin != 0U);
    if (!irq_disabled && !irq_enabled)
        return false;
    if ((config->port.ce_port == config->port.csn_port) &&
        ((config->port.ce_pin & config->port.csn_pin) != 0U))
    {
        return false;
    }

    if ((config->channel > NRF24_MAX_RF_CHANNEL) ||
        (config->data_rate > NRF24_DATA_RATE_2_MBPS) ||
        (config->output_power > NRF24_OUTPUT_POWER_0_DBM) ||
        ((config->crc_length != NRF24_CRC_1_BYTE) &&
         (config->crc_length != NRF24_CRC_2_BYTES)) ||
        (config->address_width < NRF24_MIN_ADDRESS_WIDTH) ||
        (config->address_width > NRF24_MAX_ADDRESS_WIDTH) ||
        (config->payload_size == 0U) ||
        (config->payload_size > NRF24_MAX_PAYLOAD_SIZE) ||
        (config->retransmit_count > 15U) ||
        (config->retransmit_delay_us < 250U) ||
        (config->retransmit_delay_us > 4000U) ||
        ((config->retransmit_delay_us % 250U) != 0U))
    {
        return false;
    }
    if (config->dynamic_payloads && (!config->auto_ack))
        return false;

    if ((config->data_rate == NRF24_DATA_RATE_250_KBPS) &&
        (config->retransmit_delay_us < 500U))
    {
        return false;
    }

    return true;
}

static bool Nrf24IsReady(const Nrf24_t *device)
{
    return (device != NULL) &&
           device->initialized &&
           device->port.initialized;
}

static Nrf24Result_t Nrf24ReadRegisterInternal(Nrf24_t *device,
                                               uint8_t reg,
                                               uint8_t *value)
{
    return Nrf24ReadBufferInternal(
        device,
        (uint8_t)(NRF24_CMD_R_REGISTER | (reg & NRF24_REGISTER_MASK)),
        value,
        1U);
}

static Nrf24Result_t Nrf24WriteRegisterInternal(Nrf24_t *device,
                                                uint8_t reg,
                                                uint8_t value)
{
    return Nrf24WriteBufferInternal(
        device,
        (uint8_t)(NRF24_CMD_W_REGISTER | (reg & NRF24_REGISTER_MASK)),
        &value,
        1U);
}

static Nrf24Result_t Nrf24ReadBufferInternal(Nrf24_t *device,
                                             uint8_t command,
                                             uint8_t *data,
                                             uint8_t length)
{
    uint8_t command_rx;

    if ((device == NULL) || (!device->port.initialized) ||
        (data == NULL) || (length == 0U))
    {
        return NRF24_RESULT_INVALID_ARGUMENT;
    }

    Nrf24PortSetCsn(&device->port, false);
    if (!Nrf24PortTransfer(&device->port,
                           &command,
                           &command_rx,
                           1U,
                           NRF24_CMD_NOP))
    {
        Nrf24PortSetCsn(&device->port, true);
        return NRF24_RESULT_IO_ERROR;
    }

    device->last_status = command_rx;
    if (!Nrf24PortTransfer(&device->port,
                           NULL,
                           data,
                           length,
                           NRF24_CMD_NOP))
    {
        Nrf24PortSetCsn(&device->port, true);
        return NRF24_RESULT_IO_ERROR;
    }

    Nrf24PortSetCsn(&device->port, true);
    return NRF24_RESULT_OK;
}

static Nrf24Result_t Nrf24WriteBufferInternal(Nrf24_t *device,
                                              uint8_t command,
                                              const uint8_t *data,
                                              uint8_t length)
{
    uint8_t command_rx;

    if ((device == NULL) || (!device->port.initialized) ||
        (data == NULL) || (length == 0U))
    {
        return NRF24_RESULT_INVALID_ARGUMENT;
    }

    Nrf24PortSetCsn(&device->port, false);
    if (!Nrf24PortTransfer(&device->port,
                           &command,
                           &command_rx,
                           1U,
                           NRF24_CMD_NOP))
    {
        Nrf24PortSetCsn(&device->port, true);
        return NRF24_RESULT_IO_ERROR;
    }

    device->last_status = command_rx;
    if (!Nrf24PortTransfer(&device->port,
                           data,
                           NULL,
                           length,
                           NRF24_CMD_NOP))
    {
        Nrf24PortSetCsn(&device->port, true);
        return NRF24_RESULT_IO_ERROR;
    }

    Nrf24PortSetCsn(&device->port, true);
    return NRF24_RESULT_OK;
}

static Nrf24Result_t Nrf24CommandInternal(Nrf24_t *device, uint8_t command)
{
    uint8_t status;

    if ((device == NULL) || (!device->port.initialized))
        return NRF24_RESULT_INVALID_ARGUMENT;

    Nrf24PortSetCsn(&device->port, false);
    if (!Nrf24PortTransfer(&device->port,
                           &command,
                           &status,
                           1U,
                           NRF24_CMD_NOP))
    {
        Nrf24PortSetCsn(&device->port, true);
        return NRF24_RESULT_IO_ERROR;
    }

    Nrf24PortSetCsn(&device->port, true);
    device->last_status = status;
    return NRF24_RESULT_OK;
}

static Nrf24Result_t Nrf24CommandWithDataInternal(Nrf24_t *device,
                                                  uint8_t command,
                                                  uint8_t data)
{
    uint8_t tx_data[2] = {command, data};
    uint8_t rx_data[2];

    if ((device == NULL) || (!device->port.initialized))
        return NRF24_RESULT_INVALID_ARGUMENT;

    Nrf24PortSetCsn(&device->port, false);
    if (!Nrf24PortTransfer(&device->port,
                           tx_data,
                           rx_data,
                           2U,
                           NRF24_CMD_NOP))
    {
        Nrf24PortSetCsn(&device->port, true);
        return NRF24_RESULT_IO_ERROR;
    }

    Nrf24PortSetCsn(&device->port, true);
    device->last_status = rx_data[0];
    return NRF24_RESULT_OK;
}

static Nrf24Result_t Nrf24ReadStatusInternal(Nrf24_t *device, uint8_t *status)
{
    Nrf24Result_t result;

    if (status == NULL)
        return NRF24_RESULT_INVALID_ARGUMENT;

    result = Nrf24CommandInternal(device, NRF24_CMD_NOP);
    if (result == NRF24_RESULT_OK)
        *status = device->last_status;

    return result;
}

static Nrf24Result_t Nrf24ConfigureFeatureRegisters(Nrf24_t *device)
{
    Nrf24Result_t result;
    uint8_t dynpd;
    uint8_t feature;
    uint8_t feature_readback;

    feature = device->config.dynamic_payloads ? NRF24_FEATURE_EN_DPL : 0U;
    dynpd   = device->config.dynamic_payloads ? NRF24_PIPE_0_MASK : 0U;

    result = Nrf24WriteRegisterInternal(device, NRF24_REG_FEATURE, feature);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24ReadRegisterInternal(device,
                                       NRF24_REG_FEATURE,
                                       &feature_readback);
    if (result != NRF24_RESULT_OK)
        return result;

    if ((feature_readback & 0x07U) != feature)
    {
        result = Nrf24CommandWithDataInternal(device,
                                              NRF24_CMD_ACTIVATE,
                                              NRF24_ACTIVATE_DATA);
        if (result != NRF24_RESULT_OK)
            return result;
        result = Nrf24WriteRegisterInternal(device,
                                            NRF24_REG_FEATURE,
                                            feature);
        if (result != NRF24_RESULT_OK)
            return result;
        result = Nrf24ReadRegisterInternal(device,
                                           NRF24_REG_FEATURE,
                                           &feature_readback);
        if (result != NRF24_RESULT_OK)
            return result;
        if ((feature_readback & 0x07U) != feature)
            return NRF24_RESULT_DEVICE_NOT_FOUND;
    }

    return Nrf24WriteRegisterInternal(device, NRF24_REG_DYNPD, dynpd);
}

static Nrf24Result_t Nrf24ApplyRadioConfig(Nrf24_t *device)
{
    Nrf24Result_t result;
    uint8_t config_reg;
    uint8_t rf_setup;
    uint8_t setup_retr;
    uint8_t setup_aw;
    uint8_t rx_payload_width;

    config_reg = NRF24_CONFIG_EN_CRC;
    if (device->config.crc_length == NRF24_CRC_2_BYTES)
        config_reg |= NRF24_CONFIG_CRCO;
    rf_setup        = Nrf24BuildRfSetup(device->config.data_rate,
                                        device->config.output_power);
    setup_retr      = Nrf24BuildSetupRetr(&device->config);
    setup_aw        = Nrf24AddressWidthToRegister(
        device->config.address_width);
    rx_payload_width = device->config.dynamic_payloads ?
                       0U :
                       device->config.payload_size;

    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_CONFIG,
                                        config_reg);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(
        device,
        NRF24_REG_EN_AA,
        device->config.auto_ack ? NRF24_PIPE_0_MASK : 0U);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_EN_RXADDR,
                                        NRF24_PIPE_0_MASK);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_SETUP_AW,
                                        setup_aw);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_SETUP_RETR,
                                        setup_retr);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_RF_CH,
                                        device->config.channel);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_RF_SETUP,
                                        rf_setup);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteBufferInternal(
        device,
        (uint8_t)(NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR),
        device->config.tx_address,
        device->config.address_width);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteBufferInternal(
        device,
        (uint8_t)(NRF24_CMD_W_REGISTER | NRF24_REG_RX_ADDR_P0),
        device->config.rx_address_p0,
        device->config.address_width);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_RX_PW_P0,
                                        rx_payload_width);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24ConfigureFeatureRegisters(device);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24CommandInternal(device, NRF24_CMD_FLUSH_TX);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24CommandInternal(device, NRF24_CMD_FLUSH_RX);
    if (result != NRF24_RESULT_OK)
        return result;
    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_STATUS,
                                        NRF24_STATUS_IRQ_MASK);
    if (result != NRF24_RESULT_OK)
        return result;

    result = Nrf24WriteRegisterInternal(device,
                                        NRF24_REG_CONFIG,
                                        config_reg | NRF24_CONFIG_PWR_UP);
    if (result != NRF24_RESULT_OK)
        return result;

    Nrf24PortDelayUs(NRF24_POWER_UP_DELAY_US);
    return NRF24_RESULT_OK;
}

static uint8_t Nrf24BuildRfSetup(Nrf24DataRate_t data_rate,
                                 Nrf24OutputPower_t output_power)
{
    uint8_t rf_setup = (uint8_t)((uint8_t)output_power << 1U);

    if (data_rate == NRF24_DATA_RATE_250_KBPS)
        rf_setup |= NRF24_RF_SETUP_RF_DR_LOW;
    else if (data_rate == NRF24_DATA_RATE_2_MBPS)
        rf_setup |= NRF24_RF_SETUP_RF_DR_HIGH;

    return rf_setup;
}

static uint8_t Nrf24BuildSetupRetr(const Nrf24InitConfig_t *config)
{
    uint8_t ard;

    ard = (uint8_t)((config->retransmit_delay_us / 250U) - 1U);
    return (uint8_t)((ard << NRF24_SETUP_RETR_ARD_SHIFT) |
                     (config->retransmit_count &
                      NRF24_SETUP_RETR_ARC_MASK));
}

static uint8_t Nrf24AddressWidthToRegister(uint8_t address_width)
{
    return (uint8_t)(address_width - 2U);
}

static uint32_t Nrf24TimeoutMsToUs(uint32_t timeout_ms)
{
    if (timeout_ms > (UINT32_MAX / 1000U))
        return UINT32_MAX;

    return timeout_ms * 1000U;
}
