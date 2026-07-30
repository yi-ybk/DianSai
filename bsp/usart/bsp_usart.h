/**
 * @file    bsp_usart.h
 * @brief   UART设备底层驱动接口声明
 * @details 提供MSPM0 UART的阻塞、中断和DMA收发接口。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "mspm0_hal_compat.h"

#define DEVICE_USART_CNT 3U
#define USART_RXBUFF_LIMIT 256U
#define USART_DMA_CHANNEL_INVALID 0xFFU

// 模块回调函数,用于解析协议
typedef struct USARTInstance USARTInstance;
typedef void (*usart_module_callback)(USARTInstance *);

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

/**
 * @brief UART DMA通道配置结构体
 * @note  DMA通道、UART DMA触发源和DMA中断必须先在SysConfig中配置
 */
typedef struct
{
    DMA_Regs *dma;        // MSPM0 DMA外设寄存器基地址
    uint8_t rx_channel;   // UART接收DMA通道，未使用时填USART_DMA_CHANNEL_INVALID
    uint8_t tx_channel;   // UART发送DMA通道，未使用时填USART_DMA_CHANNEL_INVALID
} USART_DMA_Config_s;

// 串口实例结构体,每个module都要包含一个实例.
struct USARTInstance
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 接收缓冲区
    uint16_t recv_buff_size;                // 接收缓冲区实际大小
    uint16_t recv_buff_index;               // 环形缓冲区写指针
    uint16_t recv_data_start;               // 新收到数据的起始下标
    uint16_t recv_data_size;                // 新收到数据的字节数
    UART_HandleTypeDef *usart_handle;       // 实例对应的UART句柄
    usart_module_callback module_callback;  // 解析收到数据的回调函数

    DMA_Regs *dma;                          // DMA外设寄存器基地址
    uint8_t rx_dma_channel;                 // 接收DMA通道
    uint8_t tx_dma_channel;                 // 发送DMA通道
    bool rx_dma_active;                     // 接收DMA是否正在运行
    bool tx_dma_active;                     // 发送DMA是否正在运行

    const uint8_t *tx_buff;                 // 中断发送缓冲区
    uint16_t tx_size;                       // 中断发送总字节数
    uint16_t tx_index;                      // 中断发送当前位置
};

/* USART初始化配置结构体 */
typedef struct
{
    uint16_t recv_buff_size;               // 接收缓冲区大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的UART句柄
    usart_module_callback module_callback; // 解析收到数据的回调函数
} USART_Init_Config_s;

/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 * @param init_config 传入串口初始化结构体
 * @return USARTInstance* 成功返回实例指针，失败返回NULL
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief 判断UART句柄是否已经被USART模块注册
 * @param usart_handle 待查询的UART句柄
 * @return bool 已注册返回true，否则返回false
 */
bool USARTIsRegistered(const UART_HandleTypeDef *usart_handle);

/**
 * @brief 初始化UART中断接收服务
 * @param instance 串口实例指针
 * @note 需要在对应UART中断服务函数中调用USARTIRQHandler
 */
void USARTServiceInit(USARTInstance *instance);

/**
 * @brief 配置一个已注册UART实例使用的DMA通道
 * @param instance 串口实例指针
 * @param config DMA通道配置结构体指针
 * @return bool 配置成功返回true，参数非法返回false
 */
bool USARTConfigureDMA(USARTInstance *instance, const USART_DMA_Config_s *config);

/**
 * @brief 启动固定长度UART DMA接收
 * @param instance 串口实例指针
 * @return bool 启动成功返回true，DMA未配置或已在接收返回false
 * @note 接收完成后，在DMA中断中调用USARTDMAIRQHandler以回调并自动重启接收
 */
bool USARTStartReceiveDMA(USARTInstance *instance);

/**
 * @brief 通过DMA发送一帧UART数据
 * @param instance 串口实例指针
 * @param send_buf 待发送数据缓冲区
 * @param send_size 待发送字节数
 * @return bool 启动成功返回true，DMA未配置、忙或参数非法返回false
 */
bool USARTSendDMA(USARTInstance *instance, const uint8_t *send_buf, uint16_t send_size);

/**
 * @brief 通过中断发送一帧UART数据
 * @param instance 串口实例指针
 * @param send_buf 待发送数据缓冲区
 * @param send_size 待发送字节数
 * @return bool 启动成功返回true，忙或参数非法返回false
 */
bool USARTSendIT(USARTInstance *instance, const uint8_t *send_buf, uint16_t send_size);

/**
 * @brief 通过调用该函数发送一帧数据
 * @param instance 串口实例指针
 * @param send_buf 待发送数据缓冲区
 * @param send_size 待发送字节数
 * @param mode 发送模式
 */
void USARTSend(USARTInstance *instance,
               uint8_t *send_buf,
               uint16_t send_size,
               USART_TRANSFER_MODE mode);

/**
 * @brief 阻塞发送UART数据，并返回底层发送结果
 * @param instance 串口实例指针
 * @param send_buf 待发送数据缓冲区
 * @param send_size 发送字节数
 * @param timeout_ms 超时时间，单位ms
 * @return true发送成功，false发送失败
 * @note timeout_ms为0时使用驱动默认超时；等待期间主动让出CPU
 */
bool USARTSendBlocking(USARTInstance *instance,
                       const uint8_t *send_buf,
                       uint16_t send_size,
                       uint32_t timeout_ms);

/**
 * @brief UART中断服务分发函数
 * @param huart 触发中断的UART句柄
 * @note 在具体UARTx_IRQHandler中调用
 */
void USARTIRQHandler(UART_HandleTypeDef *huart);

/**
 * @brief DMA传输完成服务分发函数
 * @param dma 触发中断的DMA外设寄存器基地址
 * @param channel 已完成传输的DMA通道号
 * @note 在DMA_IRQHandler中根据实际完成通道调用
 */
void USARTDMAIRQHandler(DMA_Regs *dma, uint8_t channel);

/**
 * @brief 判断串口是否准备好,用于连续或异步的IT/DMA发送
 * @param instance 要判断的串口实例
 * @return uint8_t ready返回1，busy返回0
 */
uint8_t USARTIsReady(USARTInstance *instance);

/**
 * @brief 判断串口发送是否准备好
 * @param instance 要判断的串口实例
 * @return uint8_t ready返回1，busy返回0
 */
uint8_t USARTIsTxReady(USARTInstance *instance);
