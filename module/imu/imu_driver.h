/**
 * @file    imu_driver.h
 * @brief   IMU通信驱动和数据结构声明
 * @details 处理IMU数据的接收缓冲区、解析、状态等接口
 */
#pragma once

#include "bsp_usart.h"
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
    uint8_t frame[IMU_PROTOCOL_MAX_FRAME_LEN];  // 最大原始帧数据缓存
    uint16_t frame_len;                         // 当前原始帧实际长度
    ImuVector3f_t accel;                        // 加速度数据
    ImuVector3f_t gyro;                         // 角速度数据
    ImuVector3f_t angle;                        // 角度数据
    ImuQuaternion_t quaternion;                 // 四元数数据
    uint32_t frame_count;                       // 帧计数（用于判断数据更新）
} ImuData_t;

typedef struct Imu Imu_t;  // 将struct Imu前置声明为Imu_t
typedef uint8_t (*ImuFrameParser_t)(Imu_t *imu, const uint8_t *frame, uint16_t frame_len, void *context);

typedef struct
{
    uint16_t recv_buff_size;            // UART DMA接收缓冲区大小
    UART_HandleTypeDef *usart_handle;   // UART句柄
    ImuFrameParser_t parser;            // 自定义解析函数
    void *parser_context;               // 解析函数上下文
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
    uint32_t (*get_dropped_byte_count)(Imu_t *imu);
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
 * @brief   专属IMU协议帧解析函数
 * @param   imu IMU对象指针
 * @param   frame 完整协议帧
 * @param   frame_len 帧长度
 * @param   context 用户上下文，可为NULL
 */
uint8_t Imu0FrameParse(Imu_t *imu, const uint8_t *frame, uint16_t frame_len, void *context);

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

/**
 * @brief   获取由于队列满或解析异常丢弃的字节数
 * @param   imu IMU对象指针
 * @return  uint32_t 丢弃字节总数
 */
uint32_t ImuGetDroppedByteCount(Imu_t *imu);

#define IMU_OBJECT_DEFAULT                                  \
        .init = ImuInit,                                    \
        .process   = ImuProcess,                            \
        .get_data  = ImuGetData,                            \
        .get_accel = ImuGetAccel,                           \
        .get_gyro  = ImuGetGyro,                            \
        .get_angle = ImuGetAngle,                           \
        .get_quaternion = ImuGetQuaternion,                 \
        .get_dropped_byte_count = ImuGetDroppedByteCount
