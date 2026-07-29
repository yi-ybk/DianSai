/**
 * @file    bsp_spi.c
 * @brief   SPI设备底层驱动实现
 * @details 使用MSPM0 DriverLib封装SPI阻塞传输，通过实例池管理已注册外设。
 */
#include "bsp_spi.h"
#include <string.h>

/* -------------------- 静态变量区 -------------------- */
/** @brief 已注册的SPI实例总数 */
static uint8_t idx;
/** @brief 预分配的全局SPI实例静态内存池 */
static SPIInstance spi_instance_pool[SPI_DEVICE_CNT];
/** @brief 全局注册的SPI实例指针数组 */
static SPIInstance *spi_instance[SPI_DEVICE_CNT];

SPIInstance *SPIRegister(SPI_Init_Config_s *config)
{
    SPIInstance *instance;

    if ((config == NULL) || (config->spi == NULL))
        return NULL;

    for (uint8_t i = 0U; i < idx; i++)
    {
        if (spi_instance[i]->spi == config->spi)
            return spi_instance[i];
    }

    if (idx >= SPI_DEVICE_CNT)
        return NULL;

    instance = &spi_instance_pool[idx];
    memset(instance, 0, sizeof(*instance));

    instance->spi = config->spi;
    instance->id  = config->id;

    spi_instance[idx++] = instance;
    return instance;
}

bool SPITransmit(SPIInstance *spi, const uint8_t *data, uint16_t size)
{
    return SPITransmitReceive(spi, data, NULL, size, 0U);
}

bool SPIReceive(SPIInstance *spi, uint8_t *data, uint16_t size, uint8_t fill_data)
{
    return SPITransmitReceive(spi, NULL, data, size, fill_data);
}

bool SPITransmitReceive(SPIInstance *spi,
                        const uint8_t *tx_data,
                        uint8_t *rx_data,
                        uint16_t size,
                        uint8_t fill_data)
{
    if ((spi == NULL) || (spi->spi == NULL) || (size == 0U))
        return false;

    for (uint16_t i = 0U; i < size; i++)
    {
        uint8_t received_data;
        uint8_t transmit_data = (tx_data == NULL) ? fill_data : tx_data[i];

        DL_SPI_transmitDataBlocking8(spi->spi, transmit_data);
        received_data = DL_SPI_receiveDataBlocking8(spi->spi);

        if (rx_data != NULL)
            rx_data[i] = received_data;
    }

    return true;
}
