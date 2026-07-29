/**
 * @file    nrf24l01_port_mspm0.c
 * @brief   MSPM0G3507 hardware adaptation layer for nRF24L01+
 */
#include "nrf24l01_port_mspm0.h"

#include "bsp_delay.h"
#include <stddef.h>
#include <string.h>

static bool Nrf24PortConfigIsValid(const Nrf24PortConfig_t *config);
static bool Nrf24PortSoftwareTransfer(Nrf24Port_t *port,
                                      const uint8_t *tx_data,
                                      uint8_t *rx_data,
                                      uint16_t size,
                                      uint8_t fill_data);

bool Nrf24PortInit(Nrf24Port_t *port, const Nrf24PortConfig_t *config)
{
    SPI_Init_Config_s spi_config;

    if ((port == NULL) || (!Nrf24PortConfigIsValid(config)))
        return false;

    memset(port, 0, sizeof(*port));
    memset(&spi_config, 0, sizeof(spi_config));

    port->software_spi = config->software_spi;
    if (config->software_spi)
    {
        DL_GPIO_initDigitalOutput(config->sck_iomux);
        DL_GPIO_initDigitalOutput(config->mosi_iomux);
        DL_GPIO_initDigitalInput(config->miso_iomux);
        DL_GPIO_clearPins(config->sck_port, config->sck_pin);
        DL_GPIO_clearPins(config->mosi_port, config->mosi_pin);
        DL_GPIO_enableOutput(config->sck_port, config->sck_pin);
        DL_GPIO_enableOutput(config->mosi_port, config->mosi_pin);
        port->sck_port  = config->sck_port;
        port->sck_pin   = config->sck_pin;
        port->mosi_port = config->mosi_port;
        port->mosi_pin  = config->mosi_pin;
        port->miso_port = config->miso_port;
        port->miso_pin  = config->miso_pin;
    }
    else
    {
        spi_config.spi = config->spi;
        spi_config.id  = config->id;
        port->spi      = SPIRegister(&spi_config);
        if (port->spi == NULL)
            return false;
    }

    port->ce_port  = config->ce_port;
    port->ce_pin   = config->ce_pin;
    port->csn_port = config->csn_port;
    port->csn_pin  = config->csn_pin;
    port->irq_port = config->irq_port;
    port->irq_pin  = config->irq_pin;
    port->id       = config->id;

    DL_GPIO_clearPins(port->ce_port, port->ce_pin);
    DL_GPIO_setPins(port->csn_port, port->csn_pin);
    port->initialized = true;
    return true;
}

bool Nrf24PortTransfer(Nrf24Port_t *port,
                       const uint8_t *tx_data,
                       uint8_t *rx_data,
                       uint16_t size,
                       uint8_t fill_data)
{
    if ((port == NULL) || (!port->initialized))
        return false;

    if (port->software_spi)
        return Nrf24PortSoftwareTransfer(
            port, tx_data, rx_data, size, fill_data);

    return SPITransmitReceive(port->spi, tx_data, rx_data, size, fill_data);
}

void Nrf24PortSetCe(Nrf24Port_t *port, bool high)
{
    if ((port == NULL) || (!port->initialized))
        return;

    if (high)
        DL_GPIO_setPins(port->ce_port, port->ce_pin);
    else
        DL_GPIO_clearPins(port->ce_port, port->ce_pin);
}

void Nrf24PortSetCsn(Nrf24Port_t *port, bool high)
{
    if ((port == NULL) || (!port->initialized))
        return;

    if (high)
        DL_GPIO_setPins(port->csn_port, port->csn_pin);
    else
        DL_GPIO_clearPins(port->csn_port, port->csn_pin);
}

bool Nrf24PortIrqIsActive(const Nrf24Port_t *port)
{
    if (!Nrf24PortHasIrq(port))
        return false;

    return (DL_GPIO_readPins(port->irq_port, port->irq_pin) == 0U);
}

bool Nrf24PortHasIrq(const Nrf24Port_t *port)
{
    return (port != NULL) &&
           port->initialized &&
           (port->irq_port != NULL) &&
           (port->irq_pin != 0U);
}

void Nrf24PortDelayUs(uint32_t us)
{
    BSP_DelayUs(us);
}

void Nrf24PortDelayMs(uint32_t ms)
{
    BSP_DelayMs(ms);
}

static bool Nrf24PortConfigIsValid(const Nrf24PortConfig_t *config)
{
    bool irq_disabled;
    bool irq_enabled;

    if ((config == NULL) ||
        (config->ce_port == NULL) ||
        (config->ce_pin == 0U) ||
        (config->csn_port == NULL) ||
        (config->csn_pin == 0U))
    {
        return false;
    }
    if (config->software_spi)
    {
        if ((config->sck_port == NULL) || (config->sck_pin == 0U) ||
            (config->sck_iomux == 0U) ||
            (config->mosi_port == NULL) || (config->mosi_pin == 0U) ||
            (config->mosi_iomux == 0U) ||
            (config->miso_port == NULL) || (config->miso_pin == 0U) ||
            (config->miso_iomux == 0U))
        {
            return false;
        }
    }
    else if (config->spi == NULL)
    {
        return false;
    }
    if ((config->ce_port == config->csn_port) &&
        ((config->ce_pin & config->csn_pin) != 0U))
    {
        return false;
    }

    irq_disabled = (config->irq_port == NULL) && (config->irq_pin == 0U);
    irq_enabled  = (config->irq_port != NULL) && (config->irq_pin != 0U);
    return irq_disabled || irq_enabled;
}

static bool Nrf24PortSoftwareTransfer(Nrf24Port_t *port,
                                      const uint8_t *tx_data,
                                      uint8_t *rx_data,
                                      uint16_t size,
                                      uint8_t fill_data)
{
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < size; index++)
    {
        uint8_t tx = (tx_data == NULL) ? fill_data : tx_data[index];
        uint8_t rx = 0U;

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((tx & 0x80U) != 0U)
                DL_GPIO_setPins(port->mosi_port, port->mosi_pin);
            else
                DL_GPIO_clearPins(port->mosi_port, port->mosi_pin);
            tx <<= 1U;

            Nrf24PortDelayUs(1U);
            DL_GPIO_setPins(port->sck_port, port->sck_pin);
            Nrf24PortDelayUs(1U);
            rx = (uint8_t)(rx << 1U);
            if (DL_GPIO_readPins(port->miso_port, port->miso_pin) != 0U)
                rx |= 1U;
            DL_GPIO_clearPins(port->sck_port, port->sck_pin);
            Nrf24PortDelayUs(1U);
        }

        if (rx_data != NULL)
            rx_data[index] = rx;
    }

    DL_GPIO_clearPins(port->sck_port, port->sck_pin);
    return true;
}
