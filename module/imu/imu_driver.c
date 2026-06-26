/**
 * @file    imu_driver.c
 * @brief   IMU设备层驱动实现
 * @details 负责接收串口发送来的IMU数据流，缓冲入环形队列并按协议帧进行解析。
 */
#include "imu_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

/* -------------------- 静态变量区 -------------------- */
/** @brief 已经注册并初始化的IMU设备实例数组，用于在底层串口回调时通过句柄查找对应对象 */
static Imu_t *imu_objects[DEVICE_USART_CNT];

/* ------------------ 内部静态函数区 ------------------ */

/**
 * @brief   绑定IMU对外公开的方法（接口函数指针赋值）
 * @param   imu IMU对象指针
 */
static void ImuBindMethods(Imu_t *imu)
{
    if (imu == NULL)
        return;

    imu->init = ImuInit;
    imu->process   = ImuProcess;
    imu->get_data  = ImuGetData;
    imu->get_accel = ImuGetAccel;
    imu->get_gyro  = ImuGetGyro;
    imu->get_angle = ImuGetAngle;
    imu->get_quaternion = ImuGetQuaternion;
    imu->get_dropped_byte_count = ImuGetDroppedByteCount;
}

/**
 * @brief   获取环形队列中下一个索引值
 * @param   index 当前索引
 * @return  uint16_t 下一个有效索引
 */
static uint16_t ImuQueueNextIndex(uint16_t index)
{
    index++;
    if (index >= IMU_UART_RX_QUEUE_LEN)
        index = 0;

    return index;
}

/**
 * @brief   将单个字节数据推入IMU对应的循环接收队列
 * @param   imu IMU对象指针
 * @param   byte 要存入队列的字节数据
 */
static void ImuQueuePush(Imu_t *imu, uint8_t byte)
{
    uint16_t next_write_index;

    if (imu == NULL)
        return;

    next_write_index = ImuQueueNextIndex(imu->rx_queue.write_index);
    if (next_write_index == imu->rx_queue.read_index)
    {
        imu->rx_queue.read_index = ImuQueueNextIndex(imu->rx_queue.read_index);
        imu->rx_queue.dropped_byte_count++;
    }

    imu->rx_queue.data[imu->rx_queue.write_index] = byte;
    imu->rx_queue.write_index = next_write_index;
}

static uint8_t ImuProtocolIsValid(const Imu_t *imu)
{
    if (imu == NULL)
        return 0;

    if ((imu->protocol_frame_len == 0U) ||
        (imu->protocol_frame_len > IMU_PROTOCOL_MAX_FRAME_LEN) ||
        (imu->protocol_frame_len >= IMU_UART_RX_QUEUE_LEN))
    {
        return 0;
    }

    if ((imu->protocol_header_len == 0U) ||
        (imu->protocol_tail_len == 0U) ||
        (imu->protocol_header_bytes == NULL) ||
        (imu->protocol_tail_bytes == NULL))
    {
        return 0;
    }

    if (imu->protocol_frame_len < (imu->protocol_header_len + imu->protocol_tail_len))
        return 0;

    return 1;
}

/**
 * @brief   获取IMU当前接收队列中已拥有的数据字节数
 * @param   imu IMU对象指针
 * @return  uint16_t 队列中累积的有效数据大小
 */
static uint16_t ImuQueueDataSize(Imu_t *imu)
{
    if (imu == NULL)
        return 0;

    if (imu->rx_queue.write_index >= imu->rx_queue.read_index)
        return imu->rx_queue.write_index - imu->rx_queue.read_index;

    return IMU_UART_RX_QUEUE_LEN - imu->rx_queue.read_index + imu->rx_queue.write_index;
}

/**
 * @brief   查阅（Peek）队列中某一偏移处的数据（不移动读取指针）
 * @param   imu IMU对象指针
 * @param   offset 相对于当前读指针的向后偏移量
 * @param   byte 存放读出数据的指针
 * @return  uint8_t 成功查阅返回1，越界或入参非法返回0
 */
static uint8_t ImuQueuePeek(Imu_t *imu, uint16_t offset, uint8_t *byte)
{
    uint16_t index;

    if ((imu == NULL) || (byte == NULL) || (offset >= ImuQueueDataSize(imu)))
        return 0;

    index = imu->rx_queue.read_index + offset;
    if (index >= IMU_UART_RX_QUEUE_LEN)
        index -= IMU_UART_RX_QUEUE_LEN;

    *byte = imu->rx_queue.data[index];
    return 1;
}

/**
 * @brief   从队列中丢弃指定数量的数据（移动读指针）
 * @param   imu IMU对象指针
 * @param   size 需要丢弃的字节数
 */
static void ImuQueueDiscard(Imu_t *imu, uint16_t size)
{
    if (imu == NULL)
        return;

    while ((size > 0U) && (imu->rx_queue.read_index != imu->rx_queue.write_index))
    {
        imu->rx_queue.read_index = ImuQueueNextIndex(imu->rx_queue.read_index);
        size--;
    }
}

/**
 * @brief   判断队列中从指定偏移位置开始的一段数据是否与期望的数组相等
 * @param   imu IMU对象指针
 * @param   offset 起始检查偏移量
 * @param   expected 期望的匹配字节数组
 * @param   size 匹配数组大小
 * @return  uint8_t 全部匹配成功返回1，否则返回0
 */
static uint8_t ImuQueueMatches(Imu_t *imu, uint16_t offset, const uint8_t *expected, uint16_t size)
{
    uint8_t byte;

    for (uint16_t i = 0; i < size; i++)
    {
        if (!ImuQueuePeek(imu, offset + i, &byte) || (byte != expected[i]))
            return 0;
    }

    return 1;
}

/**
 * @brief   校验整帧数据合法性（例如和校验或CRC校验等）
 * @param   frame 单帧数据缓冲指针（长度为协议预定义帧长）
 * @return  uint8_t 校验通过返回1，失败返回0
 */
static uint8_t ImuVerifyFrame(const uint8_t *frame, uint16_t frame_len)
{
    (void)frame;
    (void)frame_len;

    /* TODO: Add the IMU-specific checksum after the device protocol is selected. */
    return 1;
}

/**
 * @brief   IMU内部调用的协议帧解析代理函数
 * @param   imu IMU对象指针
 * @param   frame 单帧数据载体指针
 */
static void ImuParseFrame(Imu_t *imu, const uint8_t *frame)
{
    uint16_t frame_len;
    uint8_t parse_ok;

    if ((imu == NULL) || (frame == NULL))
        return;

    frame_len = imu->protocol_frame_len;
    parse_ok = 1;

    taskENTER_CRITICAL();
    if (imu->parser != NULL)
        parse_ok = imu->parser(imu, frame, frame_len, imu->parser_context);

    if (!parse_ok)
    {
        taskEXIT_CRITICAL();
        return;
    }

    memcpy(imu->data.frame, frame, frame_len);
    imu->data.frame_len = frame_len;
    imu->data.frame_count++;
    taskEXIT_CRITICAL();
}

/**
 * @brief   通过串口对象句柄在全局已注册设备列表中查找对应的IMU设备
 * @param   instance 串口外设实例指针
 * @return  Imu_t* 查找到的对应IMU设备指针，未找到则返回NULL
 */
static Imu_t *ImuFindObject(USARTInstance *instance)
{
    if (instance == NULL)
        return NULL;

    for (uint8_t i = 0; i < DEVICE_USART_CNT; ++i)
    {
        if (imu_objects[i] == NULL)
            continue;

        if ((imu_objects[i]->usart == instance) ||
            (imu_objects[i]->usart_handle == instance->usart_handle))
        {
            return imu_objects[i];
        }
    }

    return NULL;
}

/**
 * @brief   全局注册当前IMU设备对象
 * @param   imu 需要注册的IMU对象
 * @return  bool 注册成功返回true，失败或者列表已满返回false
 */
static bool ImuRegisterObject(Imu_t *imu)
{
    if (imu == NULL)
        return false;

    for (uint8_t i = 0; i < DEVICE_USART_CNT; ++i)
    {
        if (imu_objects[i] == imu)
            return true;
    }

    for (uint8_t i = 0; i < DEVICE_USART_CNT; ++i)
    {
        if (imu_objects[i] == NULL)
        {
            imu_objects[i] = imu;
            return true;
        }
    }

    return false;
}

/**
 * @brief   IMU设备挂载的串口通信收数完毕后的底层回调处理
 * @param   instance 发生Rx事件对应的底层串口外设实例
 */
static void ImuUARTCallback(USARTInstance *instance)
{
    Imu_t *imu = ImuFindObject(instance);

    if (imu == NULL)
        return;
    
    // 为支持环形缓冲区，不能使用memcpy
    for (uint16_t i = 0; i < instance->recv_data_size; ++i)
    {
        uint16_t index = instance->recv_data_start + i;

        if (index >= instance->recv_buff_size)
            index -= instance->recv_buff_size;

        ImuQueuePush(imu, instance->recv_buff[index]);
    }
}

/* -------------------- 接口函数区 -------------------- */

bool ImuInit(Imu_t *imu, const ImuInitConfig_t *config)
{
    USART_Init_Config_s usart_config;

    if ((imu == NULL) || (config == NULL) || (config->usart_handle == NULL))
        return false;

    if ((config->recv_buff_size == 0U) || (config->recv_buff_size > USART_RXBUFF_LIMIT))
        return false;

    ImuBindMethods(imu);
    if (imu->initialized)
        return true;

    memset(&imu->rx_queue, 0, sizeof(imu->rx_queue));
    memset(&imu->data, 0, sizeof(imu->data));
    imu->parser = config->parser;
    imu->parser_context = config->parser_context;
    imu->usart_handle = config->usart_handle;

    if (!ImuProtocolIsValid(imu))
        return false;

    if (!ImuRegisterObject(imu))
        return false;

    usart_config.recv_buff_size  = config->recv_buff_size;
    usart_config.usart_handle    = config->usart_handle;
    usart_config.module_callback = ImuUARTCallback;
    imu->usart = USARTRegister(&usart_config);
    imu->initialized = (imu->usart != NULL);

    return imu->initialized;
}

void ImuProcess(Imu_t *imu)
{
    uint8_t frame[IMU_PROTOCOL_MAX_FRAME_LEN];

    if ((imu == NULL) || (!imu->initialized) || (!ImuProtocolIsValid(imu)))
        return;

    while (ImuQueueDataSize(imu) >= imu->protocol_header_len)
    {
        if (!ImuQueueMatches(imu, 0, imu->protocol_header_bytes, imu->protocol_header_len))
        {
            ImuQueueDiscard(imu, 1);
            continue;
        }

        if(ImuQueueDataSize(imu) < imu->protocol_frame_len)
            break;

        if (!ImuQueueMatches(imu,
                             imu->protocol_frame_len - imu->protocol_tail_len,
                             imu->protocol_tail_bytes,
                             imu->protocol_tail_len))
        {
            ImuQueueDiscard(imu, 1);
            continue;
        }

        for (uint16_t i = 0; i < imu->protocol_frame_len; ++i)
            (void)ImuQueuePeek(imu, i, &frame[i]);

        ImuQueueDiscard(imu, imu->protocol_frame_len);
        if (ImuVerifyFrame(frame, imu->protocol_frame_len))
            ImuParseFrame(imu, frame);
    }
}

uint8_t Imu0FrameParse(Imu_t *imu, const uint8_t *frame, uint16_t frame_len, void *context)
{
    (void)imu;
    (void)frame;
    (void)frame_len;
    (void)context;
    /*TODO: Implement IMU frame parsing logic */
    return 1;
}

void ImuGetData(Imu_t *imu, ImuData_t *data)
{
    if ((imu != NULL) && (data != NULL))
    {
        taskENTER_CRITICAL();
        memcpy(data, &imu->data, sizeof(*data));
        taskEXIT_CRITICAL();
    }
}

void ImuGetAccel(Imu_t *imu, ImuVector3f_t *accel)
{
    if ((imu != NULL) && (accel != NULL))
    {
        taskENTER_CRITICAL();
        memcpy(accel, &imu->data.accel, sizeof(*accel));
        taskEXIT_CRITICAL();
    }
}

void ImuGetGyro(Imu_t *imu, ImuVector3f_t *gyro)
{
    if ((imu != NULL) && (gyro != NULL))
    {
        taskENTER_CRITICAL();
        memcpy(gyro, &imu->data.gyro, sizeof(*gyro));
        taskEXIT_CRITICAL();
    }
}

void ImuGetAngle(Imu_t *imu, ImuVector3f_t *angle)
{
    if ((imu != NULL) && (angle != NULL))
    {
        taskENTER_CRITICAL();
        memcpy(angle, &imu->data.angle, sizeof(*angle));
        taskEXIT_CRITICAL();
    }
}

void ImuGetQuaternion(Imu_t *imu, ImuQuaternion_t *quaternion)
{
    if ((imu != NULL) && (quaternion != NULL))
    {
        taskENTER_CRITICAL();
        memcpy(quaternion, &imu->data.quaternion, sizeof(*quaternion));
        taskEXIT_CRITICAL();
    }
}

uint32_t ImuGetDroppedByteCount(Imu_t *imu)
{
    if (imu == NULL)
        return 0;

    return imu->rx_queue.dropped_byte_count;
}
