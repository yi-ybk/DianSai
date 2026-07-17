/**
 * @file    bsp_can.h
 * @brief   CAN-FD设备底层驱动接口声明
 * @details 提供MSPM0 MCAN外设的注册、帧发送、轮询接收和回调服务接口。
 */
#pragma once

#include "mspm0_hal_compat.h"
#include <stdbool.h>
#include <stdint.h>

#define CAN_DEVICE_CNT 1U
#define CAN_FRAME_DATA_MAX_LEN 64U

/**
 * @brief CAN数据帧结构体
 * @note  length支持经典CAN的0~8字节，以及CAN-FD的12、16、20、24、32、48、64字节
 */
typedef struct
{
    uint32_t id;                             // 标准帧为11位ID，扩展帧为29位ID
    uint8_t length;                          // 有效数据字节数
    bool is_extended;                        // true为29位扩展帧，false为11位标准帧
    bool is_fd;                              // true为CAN-FD帧，false为经典CAN帧
    bool bit_rate_switch;                    // CAN-FD数据段是否启用速率切换
    uint8_t data[CAN_FRAME_DATA_MAX_LEN];    // 帧数据
} CANFrame;

typedef struct can_ins_temp CANInstance;
typedef void (*can_module_callback)(CANInstance *);

/**
 * @brief CAN实例结构体
 * @note  MCAN位时序、消息RAM、滤波器和中断由SysConfig负责配置
 */
struct can_ins_temp
{
    MCAN_Regs *mcan;                    // MSPM0 MCAN外设寄存器基地址
    uint32_t tx_buffer_index;           // 用于发送的专用Tx Buffer索引
    uint32_t rx_fifo_number;            // 用于接收的Rx FIFO编号
    CANFrame rx_frame;                  // 最近一次收到的数据帧
    can_module_callback module_callback; // 接收服务回调函数
    void *id;                           // 用户自定义ID，可用来保存额外的上下文
};

/**
 * @brief CAN设备初始化配置结构体
 */
typedef struct
{
    MCAN_Regs *mcan;                    // MSPM0 MCAN外设寄存器基地址
    uint32_t tx_buffer_index;           // 用于发送的专用Tx Buffer索引，范围0~31
    uint32_t rx_fifo_number;            // 接收FIFO，使用DL_MCAN_RX_FIFO_NUM_0或DL_MCAN_RX_FIFO_NUM_1
    can_module_callback module_callback; // 接收服务回调函数
    void *id;                           // 用户自定义ID
} CAN_Init_Config_s;

/**
 * @brief 注册一个CAN实例
 * @param config CAN初始化配置结构体指针
 * @return CANInstance* 成功返回实例指针，失败返回NULL
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 发送一帧CAN或CAN-FD数据
 * @param can CAN实例指针
 * @param frame 待发送的数据帧
 * @return bool 发送请求已写入消息RAM返回true，参数非法或发送Buffer忙返回false
 */
bool CANTransmit(CANInstance *can, const CANFrame *frame);

/**
 * @brief 轮询接收一帧CAN或CAN-FD数据
 * @param can CAN实例指针
 * @param frame 接收数据帧输出缓冲区
 * @return bool 成功接收一帧返回true，接收FIFO为空或参数非法返回false
 */
bool CANReceive(CANInstance *can, CANFrame *frame);

/**
 * @brief 执行一次CAN接收服务并调用模块回调函数
 * @param can CAN实例指针
 * @note 在MCAN接收中断中或周期任务中调用此函数
 */
void CANService(CANInstance *can);
