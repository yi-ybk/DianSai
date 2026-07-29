/**
 * @file    bsp_can.h
 * @brief   CAN设备底层驱动接口声明
 * @details 参考原工程的逻辑实例接口，使用MSPM0 MCAN DriverLib实现经典CAN收发。
 */
#pragma once

#include <ti/driverlib/driverlib.h>
#include <stdbool.h>
#include <stdint.h>

#define CAN_MX_REGISTER_CNT 16U
#define CAN_DEVICE_CNT CAN_MX_REGISTER_CNT
#define CAN_CLASSIC_DATA_MAX_LEN 8U

/** @brief CAN帧类型 */
typedef enum
{
    CAN_FRAME_STANDARD = 0,
    CAN_FRAME_EXTENDED,
} CANFrameType_t;

typedef struct can_ins_temp CANInstance;
typedef void (*can_module_callback)(CANInstance *);

/**
 * @brief CAN逻辑实例
 * @note  同一个MCAN外设可注册多个逻辑实例，每个实例独立保存收发ID和缓冲区。
 */
struct can_ins_temp
{
    MCAN_Regs *mcan;                    // MSPM0 MCAN外设寄存器基地址
    IRQn_Type irq_number;               // MCAN对应的NVIC中断号
    uint32_t tx_buffer_index;           // SysConfig配置的专用Tx Buffer索引
    uint32_t rx_fifo_number;            // SysConfig配置的Rx FIFO编号

    uint32_t tx_id;                     // 当前发送帧ID
    uint8_t tx_buff[CAN_CLASSIC_DATA_MAX_LEN];
    uint8_t tx_len;                     // 当前发送帧有效字节数

    uint32_t rx_id;                     // 期望接收的帧ID
    uint32_t rx_id_mask;                // ID匹配掩码，置1的位参与比较
    uint32_t rx_message_id;             // 最近一次收到的帧ID
    uint8_t rx_buff[CAN_CLASSIC_DATA_MAX_LEN];
    uint8_t rx_len;                     // 最近一次收到的有效字节数

    CANFrameType_t tx_frame_type;
    CANFrameType_t rx_frame_type;
    can_module_callback module_callback;
    void *id;                           // 用户自定义上下文
};

/** @brief CAN逻辑实例初始化配置 */
typedef struct
{
    MCAN_Regs *mcan;
    IRQn_Type irq_number;
    uint32_t tx_buffer_index;
    uint32_t rx_fifo_number;
    uint32_t tx_id;
    uint32_t rx_id;
    uint32_t rx_id_mask;
    CANFrameType_t tx_frame_type;
    CANFrameType_t rx_frame_type;
    can_module_callback module_callback;
    void *id;
} CAN_Init_Config_s;

/** @brief 注册一个CAN逻辑实例，失败返回NULL */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/** @brief 设置实例下一帧的有效数据长度，范围0~8 */
void CANSetDLC(CANInstance *can, uint8_t length);

/** @brief 设置实例下一帧的发送ID */
void CANSetTxId(CANInstance *can, uint32_t tx_id);

/**
 * @brief 使用实例的tx_id、tx_len和tx_buff发送一帧经典CAN数据
 * @param timeout_ms 等待专用Tx Buffer空闲的超时时间，单位ms
 */
bool CANTransmit(CANInstance *can, float timeout_ms);

/** @brief 等待实例的专用Tx Buffer完成发送 */
bool CANWaitForTxComplete(CANInstance *can, float timeout_ms);

/** @brief 读取接收FIFO，并向同一MCAN外设上的逻辑实例分发数据 */
void CANService(CANInstance *can);

/** @brief 在SysConfig生成的MCAN中断入口中调用 */
void CANIRQHandler(MCAN_Regs *mcan, uint32_t rx_fifo_number);
