/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "string.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;  // 已注册的实例数量
static USARTInstance usart_instance_pool[DEVICE_USART_CNT] = {0};  // 实例池
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};   // 实例指针数组,方便通过索引访问实例

static uint16_t USARTGetCircularRxDataSize(USARTInstance *_instance, uint16_t size)
{
    uint16_t current_index = size;
    uint16_t recv_data_size;

    if (current_index >= _instance->recv_buff_size)
        current_index = 0;

    if ((HAL_UARTEx_GetRxEventType(_instance->usart_handle) == HAL_UART_RXEVENT_TC) &&
        (current_index == _instance->recv_buff_index))
    {
        recv_data_size = _instance->recv_buff_size;
    }
    else if (current_index >= _instance->recv_buff_index)
    {
        recv_data_size = current_index - _instance->recv_buff_index;
    }
    else
    {
        recv_data_size = _instance->recv_buff_size - _instance->recv_buff_index + current_index;
    }

    _instance->recv_data_start = _instance->recv_buff_index;
    _instance->recv_data_size  = recv_data_size;
    _instance->recv_buff_index = current_index;
    return recv_data_size;
}

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo  串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *        可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size) != HAL_OK)
        return;

    _instance->recv_buff_index = 0;
    _instance->recv_data_start = 0;
    _instance->recv_data_size  = 0;
    // 关闭DMA半传输中断,只处理DMA传输完成和串口IDLE事件
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if ((init_config == NULL) ||
        (init_config->recv_buff_size == 0U) ||
        (init_config->recv_buff_size > USART_RXBUFF_LIMIT) ||
        (init_config->usart_handle == NULL))
    {
        while (1)
            LOGERROR("[bsp_usart] USART init config invalid!");
    }

    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查该串口是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    USARTInstance *instance = &usart_instance_pool[idx];
    memset(instance, 0, sizeof(*instance));

    instance->usart_handle    = init_config->usart_handle;
    instance->recv_buff_size  = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usart_instance[idx++] = instance;

    USARTServiceInit(instance);
    return instance;
}

/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1); // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsTxReady(USARTInstance *_instance)
{
    if ((_instance == NULL) || (_instance->usart_handle == NULL))
        return 0;

    // 只有当状态不是"正在发送"和"同时收发"时，才可以发送
    HAL_UART_StateTypeDef state = _instance->usart_handle->gState;
    return (state != HAL_UART_STATE_BUSY_TX) && (state != HAL_UART_STATE_BUSY_TX_RX);
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭DMA半传输中断.
 *        循环DMA模式下Size是DMA当前写入位置,需要结合上次位置计算本次新增数据.
 *
 * @param huart 发生中断的串口
 * @param Size DMA当前写入位置
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (huart->hdmarx->Init.Mode == DMA_CIRCULAR)
            {
                if ((USARTGetCircularRxDataSize(usart_instance[i], Size) > 0) &&
                    (usart_instance[i]->module_callback != NULL))
                {
                    usart_instance[i]->module_callback(usart_instance[i]);
                }
                return;
            }

            usart_instance[i]->recv_data_start = 0;
            usart_instance[i]->recv_data_size = Size;
            if (usart_instance[i]->recv_data_size > usart_instance[i]->recv_buff_size)
                usart_instance[i]->recv_data_size = usart_instance[i]->recv_buff_size;

            if (usart_instance[i]->module_callback != NULL)
                usart_instance[i]->module_callback(usart_instance[i]);

            memset(usart_instance[i]->recv_buff, 0, usart_instance[i]->recv_data_size); // 接收结束后清空buffer,对于变长数据是必要的
            USARTServiceInit(usart_instance[i]);
            return; // break the loop
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            memset(usart_instance[i]->recv_buff, 0, usart_instance[i]->recv_buff_size);
            USARTServiceInit(usart_instance[i]);
            LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
}
