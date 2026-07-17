/**
 * @file    bsp_spi.h
 * @brief   SPI设备底层驱动接口声明
 * @details 提供MSPM0 SPI外设的注册、阻塞发送、接收和全双工交换接口。
 */
#pragma once

#include "mspm0_hal_compat.h"
#include <stdbool.h>
#include <stdint.h>

#define SPI_DEVICE_CNT 2U

/**
 * @brief SPI实例结构体
 * @note  SPI时钟、工作模式、引脚复用和片选由SysConfig负责配置
 */
typedef struct spi_ins_temp
{
    SPI_Regs *spi;  // MSPM0 SPI外设寄存器基地址
    void *id;       // 用户自定义ID，可用来保存额外的上下文
} SPIInstance;

/**
 * @brief SPI设备初始化配置结构体
 */
typedef struct
{
    SPI_Regs *spi;  // MSPM0 SPI外设寄存器基地址
    void *id;       // 用户自定义ID
} SPI_Init_Config_s;

/**
 * @brief 注册一个SPI实例
 * @param config SPI初始化配置结构体指针
 * @return SPIInstance* 成功返回实例指针，失败返回NULL
 */
SPIInstance *SPIRegister(SPI_Init_Config_s *config);

/**
 * @brief 阻塞发送SPI数据
 * @param spi SPI实例指针
 * @param data 待发送数据缓冲区
 * @param size 待发送字节数
 * @return bool 发送请求有效返回true，参数非法返回false
 * @note 仅适用于已由SysConfig配置为控制器模式的SPI外设
 */
bool SPITransmit(SPIInstance *spi, const uint8_t *data, uint16_t size);

/**
 * @brief 阻塞接收SPI数据
 * @param spi SPI实例指针
 * @param data 接收数据缓冲区
 * @param size 接收字节数
 * @param fill_data 接收时发送的填充字节
 * @return bool 接收请求有效返回true，参数非法返回false
 * @note SPI接收需要控制器发送时钟，片选行为由SysConfig或上层GPIO控制
 */
bool SPIReceive(SPIInstance *spi, uint8_t *data, uint16_t size, uint8_t fill_data);

/**
 * @brief 阻塞全双工交换SPI数据
 * @param spi SPI实例指针
 * @param tx_data 发送数据缓冲区，可为NULL，此时发送fill_data
 * @param rx_data 接收数据缓冲区，可为NULL，此时丢弃接收数据
 * @param size 交换字节数
 * @param fill_data tx_data为NULL时发送的填充字节
 * @return bool 交换请求有效返回true，参数非法返回false
 */
bool SPITransmitReceive(SPIInstance *spi,
                        const uint8_t *tx_data,
                        uint8_t *rx_data,
                        uint16_t size,
                        uint8_t fill_data);
