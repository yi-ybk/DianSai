/**
 * @file    imu_driver.h
 * @brief   IMU通信驱动和数据结构声明
 * @details 处理IMU数据的接收缓冲区、解析、状态等接口
 */
#pragma once

#include "bsp_usart.h"
#include "mahony_ahrs.h"
#include <stdbool.h>
#include <stdint.h>

#define IMU_PROTOCOL_MAX_FRAME_LEN 64U

#define IMU_UART_DMA_RX_BUFFER_LEN 64U
#define IMU_UART_RX_QUEUE_LEN 256U

typedef struct
{
    float x;
    float y;
    float z;
} ImuVector3f_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} ImuQuaternion_t;

typedef struct
{
    ImuVector3f_t accel;        // 加速度数据，单位m/s^2
    ImuVector3f_t gyro;         // 角速度数据，单位rad/s
    ImuVector3f_t angle;        // 角度数据
    ImuQuaternion_t quaternion; // 四元数数据
    uint32_t frame_count;       // 解析成功帧计数（用于判断数据更新）
} ImuData_t;

typedef struct Imu Imu_t;  // 将struct Imu前置声明为Imu_t
typedef uint8_t (*ImuFrameParser_t)(Imu_t *imu, const uint8_t *frame, uint16_t frame_len, void *context);
typedef bool (*ImuDeviceInit_t)(Imu_t *imu, void *context);
typedef bool (*ImuAttitudeSolverInit_t)(Imu_t *imu, void *context);
typedef void (*ImuAttitudeSolverUpdate_t)(Imu_t *imu, void *context);

typedef struct
{
    ImuAttitudeSolverInit_t init;
    ImuAttitudeSolverUpdate_t update;
    void *context;
} ImuAttitudeSolver_t;

typedef struct
{
    float sample_period_s;
    float initial_quaternion[4];
    float quaternion_process_noise;
    float gyro_bias_process_noise;
    float accel_measure_noise;
    float fading_coefficient;
    float accel_lpf_time_constant;
    float accel_gravity_sign;
} ImuQuaternionEkfConfig_t;

typedef struct
{
    float sample_period_s;
    float proportional_gain;
    float integral_gain;
    float initial_quaternion[4];
    float accel_gravity_sign;
    MahonyAhrs_t ahrs;
} ImuMahonyConfig_t;

typedef struct
{
    uint16_t recv_buff_size;            // UART DMA接收缓冲区大小
    UART_HandleTypeDef *usart_handle;   // UART句柄
    ImuFrameParser_t parser;            // 自定义解析函数
    void *parser_context;               // 解析函数上下文
    ImuDeviceInit_t device_init;        // 具体型号的初始化函数
    void *device_context;               // 设备初始化函数上下文
} ImuInitConfig_t;

typedef struct
{
    volatile uint8_t data[IMU_UART_RX_QUEUE_LEN]; // 队列缓冲区，由USART回调写、任务读取
    volatile uint16_t write_index;         // DMA写入索引
    volatile uint16_t read_index;          // 读取索引
    volatile uint32_t dropped_byte_count;  // 因队列满丢弃的字节计数
} ImuRxQueue_t;

struct Imu
{
    USARTInstance *usart;               // 串口实例(关联底层UART驱动)
    UART_HandleTypeDef *usart_handle;   // UART旬柄
    ImuData_t data;                     // 解析后的数据
    ImuRxQueue_t rx_queue;              // 接收队列
    ImuFrameParser_t parser;            // 帧解析函数指针(适配不同协议)
    void *parser_context;               // 解析函数上下文(扩展用)
    ImuAttitudeSolver_t attitude_solver; // 可选姿态解算器
    uint8_t initialized;                // 初始化状态标记

    ImuInitConfig_t init_config;        // 初始化配置存储

    uint16_t protocol_frame_len;
    uint16_t protocol_header_len;
    uint16_t protocol_tail_len;
    const uint8_t *protocol_header_bytes;
    const uint8_t *protocol_tail_bytes;

    // 通用方法接口
    bool (*init)(Imu_t *imu, const ImuInitConfig_t *config);
    void (*process)(Imu_t *imu);  // 通用DMA数据处理接口
    void (*get_data)(Imu_t *imu, ImuData_t *data);
    void (*get_accel)(Imu_t *imu, ImuVector3f_t *accel);
    void (*get_gyro)(Imu_t *imu, ImuVector3f_t *gyro);
    void (*get_angle)(Imu_t *imu, ImuVector3f_t *angle);
    void (*get_quaternion)(Imu_t *imu, ImuQuaternion_t *quaternion);
};

/**
 * @brief   初始化IMU设备模块
 * @param   imu IMU对象指针
 * @param   config IMU初始化配置指针，包含对应串口配置和缓冲大小
 * @return  bool 成功返回true，失败返回false
 */
bool ImuInit(Imu_t *imu, const ImuInitConfig_t *config);

/**
 * @brief   IMU数据解析处理，应该放在任务的主循环中调用
 * @param   imu IMU对象指针
 */
void ImuProcess(Imu_t *imu);

/**
 * @brief   初始化IMU四元数EKF姿态解算器
 * @param   imu IMU对象指针
 * @param   context ImuQuaternionEkfConfig_t配置指针
 * @return  bool 成功返回true，配置无效或已被其他IMU占用返回false
 */
bool ImuQuaternionEkfSolverInit(Imu_t *imu, void *context);

/**
 * @brief   使用当前加速度和角速度更新四元数EKF姿态
 * @param   imu IMU对象指针
 * @param   context ImuQuaternionEkfConfig_t配置指针
 */
void ImuQuaternionEkfSolverUpdate(Imu_t *imu, void *context);

/**
 * @brief   初始化IMU Mahony姿态解算器
 * @param   imu IMU对象指针
 * @param   context ImuMahonyConfig_t配置指针
 * @return  bool 成功返回true，配置无效返回false
 */
bool ImuMahonySolverInit(Imu_t *imu, void *context);

/**
 * @brief   使用当前加速度和角速度更新Mahony姿态
 * @param   imu IMU对象指针
 * @param   context ImuMahonyConfig_t配置指针
 */
void ImuMahonySolverUpdate(Imu_t *imu, void *context);

/**
 * @brief   获取IMU最新的所有数据
 * @param   imu IMU对象指针
 * @param   data 数据存放目标结构体
 */
void ImuGetData(Imu_t *imu, ImuData_t *data);

/**
 * @brief   获取IMU三轴加速度
 * @param   imu IMU对象指针
 * @param   accel 加速度数据存放目标
 */
void ImuGetAccel(Imu_t *imu, ImuVector3f_t *accel);

/**
 * @brief   获取IMU三轴角速度
 * @param   imu IMU对象指针
 * @param   gyro 角速度数据存放目标
 */
void ImuGetGyro(Imu_t *imu, ImuVector3f_t *gyro);

/**
 * @brief   获取IMU解算后的角度信息
 * @param   imu IMU对象指针
 * @param   angle 角度数据存放目标
 */
void ImuGetAngle(Imu_t *imu, ImuVector3f_t *angle);

/**
 * @brief   获取IMU解算的四元数
 * @param   imu IMU对象指针
 * @param   quaternion 四元数存放目标
 */
void ImuGetQuaternion(Imu_t *imu, ImuQuaternion_t *quaternion);

#define IMU_QUATERNION_EKF_CONFIG_DEFAULT(_sample_period_s) \
    {                                                        \
        .sample_period_s = (_sample_period_s),                \
        .initial_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},       \
        .quaternion_process_noise = 10.0f,                    \
        .gyro_bias_process_noise = 0.001f,                    \
        .accel_measure_noise = 1000000.0f,                    \
        .fading_coefficient = 0.9996f,                        \
        .accel_lpf_time_constant = 0.0f,                      \
        .accel_gravity_sign = 1.0f                            \
    }

#define IMU_MAHONY_CONFIG_DEFAULT(_sample_period_s)       \
    {                                                      \
        .sample_period_s = (_sample_period_s),              \
        .proportional_gain = 0.5f,                          \
        .integral_gain = 0.0f,                              \
        .initial_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},     \
        .accel_gravity_sign = 1.0f                          \
    }

#define IMU_OBJECT_DEFAULT                  \
        .init           = ImuInit,          \
        .process        = ImuProcess,       \
        .get_data       = ImuGetData,       \
        .get_accel      = ImuGetAccel,      \
        .get_gyro       = ImuGetGyro,       \
        .get_angle      = ImuGetAngle,      \
        .get_quaternion = ImuGetQuaternion
