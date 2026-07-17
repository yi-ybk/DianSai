/**
 * @file    imu_driver.c
 * @brief   IMU设备层驱动实现
 * @details 负责接收串口发送来的IMU数据流，缓冲入环形队列并按协议帧进行解析。
 */
#include "imu_driver.h"
#include "QuaternionEKF.h"
#include "user_lib_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

/* -------------------- 静态变量区 -------------------- */
/** @brief 已经注册并初始化的IMU设备实例数组，用于在底层串口回调时通过句柄查找对应对象 */
static Imu_t *imu_objects[DEVICE_USART_CNT];
static Imu_t *quaternion_ekf_owner;
static float quaternion_ekf_relative_yaw_deg;

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

    if (imu->attitude_solver.update != NULL)
        imu->attitude_solver.update(imu, imu->attitude_solver.context);

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

    if ((imu->attitude_solver.init != NULL) && (imu->attitude_solver.update == NULL))
        return false;

    ImuBindMethods(imu);
    if (imu->initialized)
        return true;

    memset(&imu->rx_queue, 0, sizeof(imu->rx_queue));
    memset(&imu->data, 0, sizeof(imu->data));
    imu->parser = config->parser;
    imu->parser_context = config->parser_context;
    imu->usart_handle = config->usart_handle;
    imu->init_config = *config;

    if (!ImuProtocolIsValid(imu))
        return false;

    if (!ImuRegisterObject(imu))
        return false;

    usart_config.recv_buff_size  = config->recv_buff_size;
    usart_config.usart_handle    = config->usart_handle;
    usart_config.module_callback = ImuUARTCallback;
    imu->usart = USARTRegister(&usart_config);
    if (imu->usart == NULL)
        return false;

    if ((imu->attitude_solver.init != NULL) &&
        !imu->attitude_solver.init(imu, imu->attitude_solver.context))
    {
        return false;
    }

    if ((config->device_init != NULL) &&
        !config->device_init(imu, config->device_context))
    {
        return false;
    }

    imu->initialized = true;

    return imu->initialized;
}

bool ImuQuaternionEkfSolverInit(Imu_t *imu, void *context)
{
    ImuQuaternionEkfConfig_t *config = (ImuQuaternionEkfConfig_t *)context;
    float quaternion_norm_squared;

    if ((imu == NULL) || (config == NULL) ||
        (config->sample_period_s <= 0.0f) ||
        (config->quaternion_process_noise < 0.0f) ||
        (config->gyro_bias_process_noise < 0.0f) ||
        (config->accel_measure_noise <= 0.0f) ||
        (config->fading_coefficient <= 0.0f) ||
        (config->fading_coefficient > 1.0f) ||
        (config->accel_lpf_time_constant < 0.0f) ||
        ((config->accel_gravity_sign != 0.0f) &&
         (config->accel_gravity_sign != 1.0f) &&
         (config->accel_gravity_sign != -1.0f)))
    {
        return false;
    }

    quaternion_norm_squared = config->initial_quaternion[0] * config->initial_quaternion[0] +
                              config->initial_quaternion[1] * config->initial_quaternion[1] +
                              config->initial_quaternion[2] * config->initial_quaternion[2] +
                              config->initial_quaternion[3] * config->initial_quaternion[3];
    if (quaternion_norm_squared <= 0.0f)
        return false;

    if ((quaternion_ekf_owner != NULL) && (quaternion_ekf_owner != imu))
        return false;

    quaternion_ekf_owner = imu;
    quaternion_ekf_relative_yaw_deg = 0.0f;
    IMU_QuaternionEKF_Init(config->initial_quaternion,
                           config->quaternion_process_noise,
                           config->gyro_bias_process_noise,
                           config->accel_measure_noise,
                           config->fading_coefficient,
                           config->accel_lpf_time_constant);

    return true;
}

void ImuQuaternionEkfSolverUpdate(Imu_t *imu, void *context)
{
    ImuQuaternionEkfConfig_t *config = (ImuQuaternionEkfConfig_t *)context;
    float accel_gravity_sign;

    if ((imu == NULL) || (config == NULL) || (quaternion_ekf_owner != imu))
        return;

    accel_gravity_sign = config->accel_gravity_sign;
    if (accel_gravity_sign == 0.0f)
        accel_gravity_sign = 1.0f;

    IMU_QuaternionEKF_Update(imu->data.gyro.x,
                             imu->data.gyro.y,
                             imu->data.gyro.z,
                             imu->data.accel.x * accel_gravity_sign,
                             imu->data.accel.y * accel_gravity_sign,
                             imu->data.accel.z * accel_gravity_sign,
                             config->sample_period_s);

    quaternion_ekf_relative_yaw_deg = angle_wrap_180(
        quaternion_ekf_relative_yaw_deg +
        imu->data.gyro.z * config->sample_period_s * 57.295779513f);

    imu->data.angle.x = QEKF_INS.Roll;
    imu->data.angle.y = QEKF_INS.Pitch;
    imu->data.angle.z = quaternion_ekf_relative_yaw_deg;
    imu->data.quaternion.w = QEKF_INS.q[0];
    imu->data.quaternion.x = QEKF_INS.q[1];
    imu->data.quaternion.y = QEKF_INS.q[2];
    imu->data.quaternion.z = QEKF_INS.q[3];
}

bool ImuMahonySolverInit(Imu_t *imu, void *context)
{
    ImuMahonyConfig_t *config = (ImuMahonyConfig_t *)context;

    if ((imu == NULL) || (config == NULL) ||
        !(config->sample_period_s > 0.0f) ||
        !(config->proportional_gain >= 0.0f) ||
        !(config->integral_gain >= 0.0f) ||
        ((config->accel_gravity_sign != 0.0f) &&
         (config->accel_gravity_sign != 1.0f) &&
         (config->accel_gravity_sign != -1.0f)))
    {
        return false;
    }

    return MahonyAhrsInit(&config->ahrs, config->initial_quaternion);
}

void ImuMahonySolverUpdate(Imu_t *imu, void *context)
{
    ImuMahonyConfig_t *config = (ImuMahonyConfig_t *)context;
    float accel_gravity_sign;

    if ((imu == NULL) || (config == NULL))
        return;

    accel_gravity_sign = config->accel_gravity_sign;
    if (accel_gravity_sign == 0.0f)
        accel_gravity_sign = 1.0f;

    if (!MahonyAhrsUpdate(&config->ahrs,
                          imu->data.gyro.x,
                          imu->data.gyro.y,
                          imu->data.gyro.z,
                          imu->data.accel.x * accel_gravity_sign,
                          imu->data.accel.y * accel_gravity_sign,
                          imu->data.accel.z * accel_gravity_sign,
                          config->sample_period_s,
                          config->proportional_gain,
                          config->integral_gain))
    {
        return;
    }

    MahonyAhrsGetEulerDegrees(&config->ahrs,
                              &imu->data.angle.x,
                              &imu->data.angle.y,
                              &imu->data.angle.z);
    imu->data.quaternion.w = config->ahrs.quaternion[0];
    imu->data.quaternion.x = config->ahrs.quaternion[1];
    imu->data.quaternion.y = config->ahrs.quaternion[2];
    imu->data.quaternion.z = config->ahrs.quaternion[3];
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
