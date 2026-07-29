/**
 * @file    nrf24l01_port_mspm0.h
 * @brief   MSPM0G3507 hardware adaptation layer for nRF24L01+
 * @details SPI is provided by bsp_spi. CE, CSN and IRQ use TI DriverLib GPIO.
 */
#pragma once

#include "bsp_spi.h"
#include <stdbool.h>
#include <stdint.h>
#include <ti/driverlib/driverlib.h>

/** @brief MSPM0 hardware resources used by one nRF24L01+ module. */
typedef struct
{
    SPI_Regs *spi;          /**< SysConfig-generated SPI controller instance */
    bool software_spi;      /**< true: use GPIO mode-0 SPI instead of SPI HW */
    GPIO_Regs *sck_port;
    uint32_t sck_pin;
    uint32_t sck_iomux;
    GPIO_Regs *mosi_port;
    uint32_t mosi_pin;
    uint32_t mosi_iomux;
    GPIO_Regs *miso_port;
    uint32_t miso_pin;
    uint32_t miso_iomux;
    GPIO_Regs *ce_port;     /**< CE GPIO port */
    uint32_t ce_pin;        /**< CE GPIO pin mask */
    GPIO_Regs *csn_port;    /**< CSN GPIO port */
    uint32_t csn_pin;       /**< CSN GPIO pin mask */
    GPIO_Regs *irq_port;    /**< IRQ GPIO port; may be NULL when IRQ is unused */
    uint32_t irq_pin;       /**< IRQ GPIO pin mask; may be 0 when IRQ is unused */
    void *id;               /**< Optional user context */
} Nrf24PortConfig_t;

/** @brief Initialized MSPM0 hardware port. */
typedef struct
{
    SPIInstance *spi;
    bool software_spi;
    GPIO_Regs *sck_port;
    uint32_t sck_pin;
    GPIO_Regs *mosi_port;
    uint32_t mosi_pin;
    GPIO_Regs *miso_port;
    uint32_t miso_pin;
    GPIO_Regs *ce_port;
    uint32_t ce_pin;
    GPIO_Regs *csn_port;
    uint32_t csn_pin;
    GPIO_Regs *irq_port;
    uint32_t irq_pin;
    void *id;
    bool initialized;
} Nrf24Port_t;

bool Nrf24PortInit(Nrf24Port_t *port, const Nrf24PortConfig_t *config);
bool Nrf24PortTransfer(Nrf24Port_t *port,
                       const uint8_t *tx_data,
                       uint8_t *rx_data,
                       uint16_t size,
                       uint8_t fill_data);
void Nrf24PortSetCe(Nrf24Port_t *port, bool high);
void Nrf24PortSetCsn(Nrf24Port_t *port, bool high);
bool Nrf24PortIrqIsActive(const Nrf24Port_t *port);
bool Nrf24PortHasIrq(const Nrf24Port_t *port);
void Nrf24PortDelayUs(uint32_t us);
void Nrf24PortDelayMs(uint32_t ms);
