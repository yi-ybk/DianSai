/**
 * @file    slaver.c
 * @brief   通用从机串口通信组件实现
 * @details 在中断中搬运DMA接收数据，在任务中完成同步、分帧、校验和业务回调。
 */
#include "slaver.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
#include <string.h>

#define SLAVER_MAX_INSTANCE_COUNT DEVICE_USART_CNT
#define SLAVER_DEFAULT_TX_TIMEOUT_MS 100U

static Slaver_t *slaver_instances[SLAVER_MAX_INSTANCE_COUNT];

static void SlaverBindMethods(Slaver_t *slaver);
static bool SlaverConfigIsValid(const SlaverInitConfig_t *config);
static bool SlaverRegisterInstance(Slaver_t *slaver);
static Slaver_t *SlaverFindByUsart(const USARTInstance *usart);
static void SlaverUsartCallback(USARTInstance *usart);
static void SlaverPushFromIsr(Slaver_t *slaver, const uint8_t *data, uint16_t length,
                              BaseType_t *task_woken);
static void SlaverAppend(Slaver_t *slaver, const uint8_t *data, uint16_t length);
static void SlaverParse(Slaver_t *slaver);
static uint16_t SlaverFindHeader(const Slaver_t *slaver);
static uint16_t SlaverHeaderPrefixLength(const Slaver_t *slaver);
static void SlaverDropPrefix(Slaver_t *slaver, uint16_t length);
static void SlaverCopyData(Slaver_t *slaver, SlaverData_t *data);

/**
 * @brief   初始化从机串口通信对象
 * @param   slaver 从机通信对象指针
 * @param   config 初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool SlaverInit(Slaver_t *slaver, const SlaverInitConfig_t *config)
{
    USART_Init_Config_s usart_config;
    uint32_t primask;

    if ((slaver == NULL) || (config == NULL))
        return false;

    SlaverBindMethods(slaver);
    if (slaver->initialized)
        return true;
    if (!SlaverConfigIsValid(config))
        return false;

    slaver->init_config = *config;
    if (slaver->init_config.tx_timeout_ms == 0U)
        slaver->init_config.tx_timeout_ms = SLAVER_DEFAULT_TX_TIMEOUT_MS;

    memset(&slaver->data, 0, sizeof(slaver->data));
    memset(slaver->parse_buffer, 0, sizeof(slaver->parse_buffer));
    slaver->parse_length = 0U;
    slaver->rx_stream = xStreamBufferCreateStatic(SLAVER_RX_STREAM_SIZE,
                                                    1U,
                                                    slaver->rx_stream_storage,
                                                    &slaver->rx_stream_control_block);
    if (slaver->rx_stream == NULL)
        return false;

    memset(&usart_config, 0, sizeof(usart_config));
    usart_config.recv_buff_size  = slaver->init_config.dma_rx_buffer_size;
    usart_config.usart_handle    = slaver->init_config.uart_handle;
    usart_config.module_callback = SlaverUsartCallback;

    primask = __get_PRIMASK();
    __disable_irq();
    slaver->usart = USARTRegister(&usart_config);
    if ((slaver->usart == NULL) || (!SlaverRegisterInstance(slaver)))
    {
        if (primask == 0U)
            __enable_irq();
        return false;
    }
    if (primask == 0U)
        __enable_irq();

    slaver->initialized = true;
    return true;
}

/**
 * @brief   处理已经接收到的串口数据
 * @param   slaver 从机通信对象指针
 * @note    此函数必须在任务上下文调用。
 */
void SlaverProcess(Slaver_t *slaver)
{
    uint8_t buffer[64];
    size_t received;

    if ((slaver == NULL) || (!slaver->initialized) || (slaver->rx_stream == NULL))
        return;

    do
    {
        received = xStreamBufferReceive(slaver->rx_stream,
                                        buffer,
                                        sizeof(buffer),
                                        0U);
        if (received > 0U)
            SlaverAppend(slaver, buffer, (uint16_t)received);
    } while (received > 0U);
}

/**
 * @brief   从机通信任务入口
 * @param   argument Slaver_t对象指针
 */
void SlaverTask(void *argument)
{
    Slaver_t *slaver = (Slaver_t *)argument;

    if (slaver == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    for (;;)
    {
        SlaverProcess(slaver);
        vTaskDelay(pdMS_TO_TICKS(SLAVER_TASK_DELAY_MS));
    }
}

/**
 * @brief   阻塞发送完整原始帧
 * @param   slaver 从机通信对象指针
 * @param   data 待发送帧数据
 * @param   length 待发送帧长度
 * @return  bool 成功返回true，失败返回false
 */
bool SlaverSend(Slaver_t *slaver, const uint8_t *data, uint16_t length)
{
    bool result;

    if ((slaver == NULL) || (!slaver->initialized) || (data == NULL) || (length == 0U))
        return false;

    result = USARTSendBlocking(slaver->usart,
                               data,
                               length,
                               slaver->init_config.tx_timeout_ms);
    if (result)
        slaver->data.tx_frame_count++;
    else
        slaver->data.tx_error_count++;
    return result;
}

/**
 * @brief   获取从机通信运行数据的一致快照
 * @param   slaver 从机通信对象指针
 * @param   data 数据输出目标
 */
void SlaverGetData(Slaver_t *slaver, SlaverData_t *data)
{
    if ((slaver == NULL) || (data == NULL))
        return;

    SlaverCopyData(slaver, data);
}

/**
 * @brief   根据长度字段解析完整帧长度
 * @param   frame 当前候选帧数据
 * @param   available 当前可用字节数
 * @param   frame_length 输出完整帧长度
 * @param   context SlaverLengthFieldConfig_t配置指针
 * @return  SlaverFrameState_t 解析结果
 */
SlaverFrameState_t SlaverFrameLengthFromLengthField(const uint8_t *frame,
                                                     uint16_t available,
                                                     uint16_t *frame_length,
                                                     void *context)
{
    const SlaverLengthFieldConfig_t *config = (const SlaverLengthFieldConfig_t *)context;
    uint16_t length_value;
    uint32_t total_length;

    if ((frame == NULL) || (frame_length == NULL) || (config == NULL) ||
        ((config->length_size != 1U) && (config->length_size != 2U)) ||
        (config->max_frame_length == 0U))
        return SLAVER_FRAME_INVALID;

    if (available < (uint16_t)(config->length_offset + config->length_size))
        return SLAVER_FRAME_WAIT;

    if (config->length_size == 1U)
    {
        length_value = frame[config->length_offset];
    }
    else if (config->byte_order == SLAVER_BYTE_ORDER_LITTLE_ENDIAN)
    {
        length_value = (uint16_t)frame[config->length_offset] |
                       ((uint16_t)frame[config->length_offset + 1U] << 8U);
    }
    else
    {
        length_value = ((uint16_t)frame[config->length_offset] << 8U) |
                       (uint16_t)frame[config->length_offset + 1U];
    }

    total_length = config->length_is_payload ?
                   (uint32_t)length_value + config->frame_overhead : length_value;
    if ((total_length == 0U) || (total_length > config->max_frame_length) ||
        (total_length > SLAVER_PARSE_BUFFER_SIZE))
        return SLAVER_FRAME_INVALID;

    *frame_length = (uint16_t)total_length;
    return (available >= *frame_length) ? SLAVER_FRAME_READY : SLAVER_FRAME_WAIT;
}

/** @brief 绑定从机通信对象方法 */
static void SlaverBindMethods(Slaver_t *slaver)
{
    if (slaver == NULL)
        return;

    slaver->init     = SlaverInit;
    slaver->process  = SlaverProcess;
    slaver->task     = SlaverTask;
    slaver->send     = SlaverSend;
    slaver->get_data = SlaverGetData;
}

/** @brief 校验从机通信初始化配置 */
static bool SlaverConfigIsValid(const SlaverInitConfig_t *config)
{
    if ((config == NULL) || (config->uart_handle == NULL) ||
        USARTIsRegistered(config->uart_handle) ||
        (config->dma_rx_buffer_size == 0U) ||
        (config->dma_rx_buffer_size > USART_RXBUFF_LIMIT) ||
        (config->header_length == 0U) ||
        (config->header_length > SLAVER_MAX_HEADER_LENGTH) ||
        (config->frame_length_callback == NULL) ||
        (config->frame_callback == NULL))
        return false;

    return true;
}

/** @brief 注册从机通信对象与底层USART实例的对应关系 */
static bool SlaverRegisterInstance(Slaver_t *slaver)
{
    uint8_t index;

    for (index = 0U; index < SLAVER_MAX_INSTANCE_COUNT; index++)
    {
        if (slaver_instances[index] == slaver)
            return true;
        if (slaver_instances[index] == NULL)
        {
            slaver_instances[index] = slaver;
            return true;
        }
    }
    return false;
}

/** @brief 根据USART实例查找从机通信对象 */
static Slaver_t *SlaverFindByUsart(const USARTInstance *usart)
{
    uint8_t index;

    for (index = 0U; index < SLAVER_MAX_INSTANCE_COUNT; index++)
    {
        if ((slaver_instances[index] != NULL) &&
            (slaver_instances[index]->usart == usart))
            return slaver_instances[index];
    }
    return NULL;
}

/** @brief 串口DMA接收回调，仅在中断中搬运新增数据 */
static void SlaverUsartCallback(USARTInstance *usart)
{
    Slaver_t *slaver;
    uint16_t first_length;
    BaseType_t task_woken = pdFALSE;

    if (usart == NULL)
        return;

    slaver = SlaverFindByUsart(usart);
    if ((slaver == NULL) || (!slaver->initialized) || (usart->recv_data_size == 0U))
        return;

    first_length = usart->recv_data_size;
    if ((uint32_t)usart->recv_data_start + first_length > usart->recv_buff_size)
        first_length = (uint16_t)(usart->recv_buff_size - usart->recv_data_start);

    SlaverPushFromIsr(slaver,
                      &usart->recv_buff[usart->recv_data_start],
                      first_length,
                      &task_woken);
    if (usart->recv_data_size > first_length)
    {
        SlaverPushFromIsr(slaver,
                          usart->recv_buff,
                          (uint16_t)(usart->recv_data_size - first_length),
                          &task_woken);
    }

    slaver->data.rx_byte_count += usart->recv_data_size;
    slaver->data.last_rx_tick = HAL_GetTick();
    portYIELD_FROM_ISR(task_woken);
}

/** @brief 将一段DMA接收数据写入流缓冲 */
static void SlaverPushFromIsr(Slaver_t *slaver, const uint8_t *data, uint16_t length,
                              BaseType_t *task_woken)
{
    size_t sent;

    if ((slaver == NULL) || (data == NULL) || (length == 0U) || (task_woken == NULL))
        return;

    sent = xStreamBufferSendFromISR(slaver->rx_stream,
                                    data,
                                    length,
                                    task_woken);
    if (sent != length)
    {
        slaver->data.rx_stream_drop_count++;
        slaver->data.rx_stream_drop_bytes += (uint32_t)(length - sent);
    }
}

/** @brief 追加数据并解析所有可用完整帧 */
static void SlaverAppend(Slaver_t *slaver, const uint8_t *data, uint16_t length)
{
    uint16_t copy_length;

    while (length > 0U)
    {
        if (slaver->parse_length >= SLAVER_PARSE_BUFFER_SIZE)
        {
            SlaverParse(slaver);
            if (slaver->parse_length >= SLAVER_PARSE_BUFFER_SIZE)
            {
                SlaverDropPrefix(slaver, 1U);
                slaver->data.invalid_frame_count++;
            }
        }

        copy_length = (uint16_t)(SLAVER_PARSE_BUFFER_SIZE - slaver->parse_length);
        if (copy_length > length)
            copy_length = length;
        memcpy(&slaver->parse_buffer[slaver->parse_length], data, copy_length);
        slaver->parse_length = (uint16_t)(slaver->parse_length + copy_length);
        data += copy_length;
        length = (uint16_t)(length - copy_length);
        SlaverParse(slaver);
    }
}

/** @brief 从缓存中同步、校验并分发完整帧 */
static void SlaverParse(Slaver_t *slaver)
{
    SlaverFrameState_t state;
    uint16_t header_index;
    uint16_t frame_length;
    uint16_t keep_length;

    for (;;)
    {
        if (slaver->parse_length < slaver->init_config.header_length)
            return;

        header_index = SlaverFindHeader(slaver);
        if (header_index >= slaver->parse_length)
        {
            keep_length = SlaverHeaderPrefixLength(slaver);
            if (keep_length < slaver->parse_length)
            {
                slaver->data.invalid_frame_count++;
                if (keep_length > 0U)
                    memmove(slaver->parse_buffer,
                            &slaver->parse_buffer[slaver->parse_length - keep_length],
                            keep_length);
                slaver->parse_length = keep_length;
            }
            return;
        }
        if (header_index > 0U)
        {
            SlaverDropPrefix(slaver, header_index);
            slaver->data.invalid_frame_count++;
            continue;
        }

        frame_length = 0U;
        state = slaver->init_config.frame_length_callback(slaver->parse_buffer,
                                                          slaver->parse_length,
                                                          &frame_length,
                                                          slaver->init_config.protocol_context);
        if (state == SLAVER_FRAME_WAIT)
            return;
        if ((state != SLAVER_FRAME_READY) ||
            (frame_length < slaver->init_config.header_length) ||
            (frame_length > slaver->parse_length))
        {
            SlaverDropPrefix(slaver, 1U);
            slaver->data.invalid_frame_count++;
            continue;
        }

        if ((slaver->init_config.frame_validate_callback != NULL) &&
            (!slaver->init_config.frame_validate_callback(slaver->parse_buffer,
                                                          frame_length,
                                                          slaver->init_config.protocol_context)))
        {
            SlaverDropPrefix(slaver, 1U);
            slaver->data.validate_error_count++;
            continue;
        }

        slaver->init_config.frame_callback(slaver,
                                           slaver->parse_buffer,
                                           frame_length,
                                           slaver->init_config.protocol_context);
        SlaverDropPrefix(slaver, frame_length);
        slaver->data.rx_frame_count++;
        slaver->data.last_frame_tick = HAL_GetTick();
    }
}

/** @brief 查找缓存中第一个完整帧头 */
static uint16_t SlaverFindHeader(const Slaver_t *slaver)
{
    uint16_t index;

    for (index = 0U;
         (uint16_t)(index + slaver->init_config.header_length) <= slaver->parse_length;
         index++)
    {
        if (memcmp(&slaver->parse_buffer[index],
                   slaver->init_config.header,
                   slaver->init_config.header_length) == 0)
            return index;
    }
    return slaver->parse_length;
}

/** @brief 获取缓存尾部与帧头前缀匹配的最大长度 */
static uint16_t SlaverHeaderPrefixLength(const Slaver_t *slaver)
{
    uint16_t prefix_length;

    for (prefix_length = slaver->init_config.header_length - 1U;
         prefix_length > 0U;
         prefix_length--)
    {
        if ((prefix_length <= slaver->parse_length) &&
            (memcmp(&slaver->parse_buffer[slaver->parse_length - prefix_length],
                    slaver->init_config.header,
                    prefix_length) == 0))
            return prefix_length;
    }
    return 0U;
}

/** @brief 从待解析缓存中移除前缀数据 */
static void SlaverDropPrefix(Slaver_t *slaver, uint16_t length)
{
    if (length >= slaver->parse_length)
    {
        slaver->parse_length = 0U;
        return;
    }

    memmove(slaver->parse_buffer,
            &slaver->parse_buffer[length],
            slaver->parse_length - length);
    slaver->parse_length = (uint16_t)(slaver->parse_length - length);
}

/** @brief 复制从机通信运行数据 */
static void SlaverCopyData(Slaver_t *slaver, SlaverData_t *data)
{
    taskENTER_CRITICAL();
    memcpy(data, &slaver->data, sizeof(*data));
    taskEXIT_CRITICAL();
}
