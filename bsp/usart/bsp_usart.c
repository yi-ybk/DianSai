/**
 * @file    bsp_usart.c
 * @brief   UART设备底层驱动实现
 * @details 使用MSPM0 DriverLib实现UART阻塞、中断和DMA收发。
 */
#include "bsp_usart.h"
#include <string.h>

#define USART_BLOCKING_DEFAULT_TIMEOUT_MS 100U

/* -------------------- 静态变量区 -------------------- */
/** @brief 已注册的UART实例总数 */
static uint8_t idx;
/** @brief 预分配的全局UART实例静态内存池 */
static USARTInstance usart_instance_pool[DEVICE_USART_CNT];
/** @brief 全局注册的UART实例指针数组 */
static USARTInstance *usart_instance[DEVICE_USART_CNT];

static void USARTReceiveIRQ(USARTInstance *instance);
static void USARTTransmitIRQ(USARTInstance *instance);
static void USARTDMAReceiveIdle(USARTInstance *instance);
static void USARTDMAReceiveComplete(USARTInstance *instance);
static void USARTDMATransmitComplete(USARTInstance *instance);

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    USARTInstance *instance;

    if ((init_config == NULL) ||
        (init_config->recv_buff_size == 0U) ||
        (init_config->recv_buff_size > USART_RXBUFF_LIMIT) ||
        (init_config->usart_handle == NULL) ||
        (init_config->usart_handle->Instance == NULL))
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < idx; i++)
    {
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            return usart_instance[i];
    }

    if (idx >= DEVICE_USART_CNT)
        return NULL;

    instance = &usart_instance_pool[idx];
    memset(instance, 0, sizeof(*instance));

    instance->recv_buff_size  = init_config->recv_buff_size;
    instance->usart_handle    = init_config->usart_handle;
    instance->module_callback = init_config->module_callback;
    instance->rx_dma_channel  = USART_DMA_CHANNEL_INVALID;
    instance->tx_dma_channel  = USART_DMA_CHANNEL_INVALID;

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

bool USARTIsRegistered(const UART_HandleTypeDef *usart_handle)
{
    uint8_t instance_index;

    if (usart_handle == NULL)
        return false;

    for (instance_index = 0U; instance_index < idx; ++instance_index)
    {
        if (usart_instance[instance_index]->usart_handle == usart_handle)
            return true;
    }
    return false;
}

void USARTServiceInit(USARTInstance *instance)
{
    if ((instance == NULL) || (instance->usart_handle == NULL) ||
        (instance->usart_handle->Instance == NULL) || instance->rx_dma_active)
    {
        return;
    }

    instance->recv_buff_index = 0U;
    instance->recv_data_start = 0U;
    instance->recv_data_size  = 0U;
    DL_UART_enableInterrupt(instance->usart_handle->Instance, DL_UART_INTERRUPT_RX);
}

bool USARTConfigureDMA(USARTInstance *instance, const USART_DMA_Config_s *config)
{
    if ((instance == NULL) || (config == NULL) || (config->dma == NULL) ||
        ((config->rx_channel == USART_DMA_CHANNEL_INVALID) &&
         (config->tx_channel == USART_DMA_CHANNEL_INVALID)))
    {
        return false;
    }

    instance->dma            = config->dma;
    instance->rx_dma_channel = config->rx_channel;
    instance->tx_dma_channel = config->tx_channel;
    return true;
}

bool USARTStartReceiveDMA(USARTInstance *instance)
{
    if ((instance == NULL) || (instance->usart_handle == NULL) ||
        (instance->usart_handle->Instance == NULL) || (instance->dma == NULL) ||
        (instance->rx_dma_channel == USART_DMA_CHANNEL_INVALID) ||
        instance->rx_dma_active)
    {
        return false;
    }

    DL_DMA_disableChannel(instance->dma, instance->rx_dma_channel);
    DL_DMA_setSrcAddr(instance->dma,
                      instance->rx_dma_channel,
                      (uint32_t)&instance->usart_handle->Instance->RXDATA);
    DL_DMA_setDestAddr(instance->dma,
                       instance->rx_dma_channel,
                       (uint32_t)&instance->recv_buff[0]);
    DL_DMA_setTransferSize(instance->dma,
                           instance->rx_dma_channel,
                           instance->recv_buff_size);
    DL_UART_enableDMAReceiveEvent(instance->usart_handle->Instance,
                                  DL_UART_DMA_INTERRUPT_RX);
    DL_UART_disableInterrupt(instance->usart_handle->Instance, DL_UART_INTERRUPT_RX);
    DL_UART_enableInterrupt(instance->usart_handle->Instance,
                            DL_UART_INTERRUPT_DMA_DONE_RX |
                            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    DL_DMA_enableChannel(instance->dma, instance->rx_dma_channel);

    instance->rx_dma_active = true;
    return true;
}

bool USARTSendDMA(USARTInstance *instance, const uint8_t *send_buf, uint16_t send_size)
{
    if ((instance == NULL) || (instance->usart_handle == NULL) ||
        (instance->usart_handle->Instance == NULL) || (instance->dma == NULL) ||
        (instance->tx_dma_channel == USART_DMA_CHANNEL_INVALID) ||
        (send_buf == NULL) || (send_size == 0U) || instance->tx_dma_active ||
        (instance->tx_index < instance->tx_size))
    {
        return false;
    }

    DL_DMA_disableChannel(instance->dma, instance->tx_dma_channel);
    DL_DMA_setSrcAddr(instance->dma,
                      instance->tx_dma_channel,
                      (uint32_t)send_buf);
    DL_DMA_setDestAddr(instance->dma,
                       instance->tx_dma_channel,
                       (uint32_t)&instance->usart_handle->Instance->TXDATA);
    DL_DMA_setTransferSize(instance->dma, instance->tx_dma_channel, send_size);
    DL_UART_enableDMATransmitEvent(instance->usart_handle->Instance);
    DL_DMA_enableChannel(instance->dma, instance->tx_dma_channel);

    instance->tx_dma_active = true;
    instance->usart_handle->gState = HAL_UART_STATE_BUSY_TX;
    return true;
}

bool USARTSendIT(USARTInstance *instance, const uint8_t *send_buf, uint16_t send_size)
{
    if ((instance == NULL) || (instance->usart_handle == NULL) ||
        (instance->usart_handle->Instance == NULL) || (send_buf == NULL) ||
        (send_size == 0U) || instance->tx_dma_active ||
        (instance->tx_index < instance->tx_size))
    {
        return false;
    }

    instance->tx_buff  = send_buf;
    instance->tx_size  = send_size;
    instance->tx_index = 0U;
    instance->usart_handle->gState = HAL_UART_STATE_BUSY_TX;
    DL_UART_enableInterrupt(instance->usart_handle->Instance, DL_UART_INTERRUPT_TX);
    return true;
}

void USARTSend(USARTInstance *instance,
               uint8_t *send_buf,
               uint16_t send_size,
               USART_TRANSFER_MODE mode)
{
    if ((instance == NULL) || (send_buf == NULL) || (send_size == 0U))
        return;

    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        (void)USARTSendBlocking(instance, send_buf, send_size, 0U);
        break;
    case USART_TRANSFER_IT:
        (void)USARTSendIT(instance, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        (void)USARTSendDMA(instance, send_buf, send_size);
        break;
    default:
        break;
    }
}

bool USARTSendBlocking(USARTInstance *instance,
                       const uint8_t *send_buf,
                       uint16_t send_size,
                       uint32_t timeout_ms)
{
    UART_Regs *uart;
    TickType_t start_tick;
    TickType_t timeout_ticks;

    if ((instance == NULL) || (instance->usart_handle == NULL) ||
        (instance->usart_handle->Instance == NULL) || (send_buf == NULL) ||
        (send_size == 0U) || !USARTIsReady(instance))
    {
        return false;
    }

    uart = instance->usart_handle->Instance;
    if (timeout_ms == 0U)
        timeout_ms = USART_BLOCKING_DEFAULT_TIMEOUT_MS;
    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks == 0U)
        timeout_ticks = 1U;
    start_tick = xTaskGetTickCount();

    for (uint16_t i = 0U; i < send_size; i++)
    {
        while (DL_UART_isTXFIFOFull(uart))
        {
            if ((xTaskGetTickCount() - start_tick) >= timeout_ticks)
                return false;
            if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
                taskYIELD();
        }
        DL_UART_transmitData(uart, send_buf[i]);
    }

    while (!DL_UART_isTXFIFOEmpty(uart))
    {
        if ((xTaskGetTickCount() - start_tick) >= timeout_ticks)
            return false;
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
            taskYIELD();
    }

    return true;
}

void USARTIRQHandler(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance == NULL))
        return;

    for (uint8_t i = 0U; i < idx; i++)
    {
        USARTInstance *instance = usart_instance[i];
        DL_UART_IIDX interrupt;

        if ((instance == NULL) || (instance->usart_handle != huart))
            continue;

        interrupt = DL_UART_getPendingInterrupt(huart->Instance);
        if (interrupt == DL_UART_IIDX_RX)
            USARTReceiveIRQ(instance);
        else if (interrupt == DL_UART_IIDX_TX)
            USARTTransmitIRQ(instance);
        else if (interrupt == DL_UART_IIDX_DMA_DONE_RX)
            USARTDMAReceiveComplete(instance);
        else if (interrupt == DL_UART_IIDX_RX_TIMEOUT_ERROR)
            USARTDMAReceiveIdle(instance);
        else if (interrupt != DL_UART_IIDX_NO_INTERRUPT)
            DL_UART_clearInterruptStatus(huart->Instance,
                                         DL_UART_getEnabledInterruptStatus(huart->Instance, 0xFFFFFFFFU));
        return;
    }
}

void USARTDMAIRQHandler(DMA_Regs *dma, uint8_t channel)
{
    for (uint8_t i = 0U; i < idx; i++)
    {
        USARTInstance *instance = usart_instance[i];

        if ((instance == NULL) || (instance->dma != dma))
            continue;

        if (instance->rx_dma_active && (instance->rx_dma_channel == channel))
        {
            USARTDMAReceiveComplete(instance);
            return;
        }

        if (instance->tx_dma_active && (instance->tx_dma_channel == channel))
        {
            USARTDMATransmitComplete(instance);
            return;
        }
    }
}

uint8_t USARTIsReady(USARTInstance *instance)
{
    if ((instance == NULL) || (instance->usart_handle == NULL))
        return 0U;

    return (!instance->tx_dma_active && (instance->tx_index >= instance->tx_size)) ? 1U : 0U;
}

uint8_t USARTIsTxReady(USARTInstance *instance)
{
    return USARTIsReady(instance);
}

static void USARTReceiveIRQ(USARTInstance *instance)
{
    uint16_t received_count = 0U;

    instance->recv_data_start = instance->recv_buff_index;
    while (!DL_UART_isRXFIFOEmpty(instance->usart_handle->Instance))
    {
        instance->recv_buff[instance->recv_buff_index] =
            DL_UART_receiveData(instance->usart_handle->Instance);
        instance->recv_buff_index++;
        if (instance->recv_buff_index >= instance->recv_buff_size)
            instance->recv_buff_index = 0U;
        received_count++;
    }

    instance->recv_data_size = received_count;
    if ((received_count > 0U) && (instance->module_callback != NULL))
        instance->module_callback(instance);
}

static void USARTTransmitIRQ(USARTInstance *instance)
{
    while ((instance->tx_index < instance->tx_size) &&
           !DL_UART_isTXFIFOFull(instance->usart_handle->Instance))
    {
        DL_UART_transmitData(instance->usart_handle->Instance,
                             instance->tx_buff[instance->tx_index++]);
    }

    if (instance->tx_index >= instance->tx_size)
    {
        DL_UART_disableInterrupt(instance->usart_handle->Instance, DL_UART_INTERRUPT_TX);
        instance->tx_buff  = NULL;
        instance->tx_size  = 0U;
        instance->tx_index = 0U;
        instance->usart_handle->gState = 0U;
    }
}

static void USARTDMAReceiveIdle(USARTInstance *instance)
{
    uint16_t remaining;
    uint16_t received;

    if ((instance == NULL) || (!instance->rx_dma_active) ||
        (instance->dma == NULL) ||
        (instance->rx_dma_channel == USART_DMA_CHANNEL_INVALID))
    {
        return;
    }

    DL_DMA_disableChannel(instance->dma, instance->rx_dma_channel);
    remaining = DL_DMA_getTransferSize(instance->dma, instance->rx_dma_channel);
    if (remaining > instance->recv_buff_size)
        remaining = instance->recv_buff_size;

    received = instance->recv_buff_size - remaining;
    instance->rx_dma_active = false;

    if (received > 0U)
    {
        instance->recv_data_start = 0U;
        instance->recv_data_size  = received;
        instance->recv_buff_index = 0U;

        if (instance->module_callback != NULL)
            instance->module_callback(instance);
    }

    (void)USARTStartReceiveDMA(instance);
}

static void USARTDMAReceiveComplete(USARTInstance *instance)
{
    DL_DMA_disableChannel(instance->dma, instance->rx_dma_channel);
    instance->rx_dma_active  = false;
    instance->recv_data_start = 0U;
    instance->recv_data_size  = instance->recv_buff_size;
    instance->recv_buff_index = 0U;

    if (instance->module_callback != NULL)
        instance->module_callback(instance);

    (void)USARTStartReceiveDMA(instance);
}

static void USARTDMATransmitComplete(USARTInstance *instance)
{
    DL_DMA_disableChannel(instance->dma, instance->tx_dma_channel);
    DL_UART_disableDMATransmitEvent(instance->usart_handle->Instance);
    instance->tx_dma_active = false;
    instance->usart_handle->gState = 0U;
}
