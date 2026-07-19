#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef HAL_CAN_MODULE_ENABLED
#include "can.h"
typedef CAN_HandleTypeDef CAN_HardwareHandle_t;
typedef CAN_TxHeaderTypeDef CAN_TxHeader_t;
#else
typedef void CAN_HardwareHandle_t;
typedef struct
{
    uint8_t reserved;
} CAN_TxHeader_t;
#endif

// 最多能够支持的CAN设备数
#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 2           // 最多支持的CAN外设句柄数量

/** @brief CAN帧类型 */
typedef enum
{
    CAN_FRAME_STANDARD = 0, /**< 标准帧 */
    CAN_FRAME_EXTENDED,     /**< 扩展帧 */
} CANFrameType_t;

/* can instance typedef, every module registered to CAN should have this variable */
#pragma pack(1)
typedef struct _
{
    CAN_HardwareHandle_t *can_handle; // can句柄
    CAN_TxHeader_t txconf;            // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint32_t rx_id_mask;           // 接收id掩码，置1的位参与匹配
    uint32_t rx_message_id;        // 最近一次实际接收的报文id
    uint8_t rx_len;                // 接收长度,可能为0-8
    CANFrameType_t tx_frame_type;  // 发送帧类型
    CANFrameType_t rx_frame_type;  // 接收帧类型
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;
#pragma pack()

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HardwareHandle_t *can_handle;           // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    uint32_t rx_id_mask;                        // 接收id掩码，0表示精确匹配
    CANFrameType_t tx_frame_type;               // 发送帧类型，默认标准帧
    CANFrameType_t rx_frame_type;               // 接收帧类型，默认标准帧
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        注册(初始化)一个can实例,需要传入初始化配置的指针.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief 修改CAN发送报文ID
 * @param _instance CAN实例
 * @param tx_id 发送ID
 */
void CANSetTxId(CANInstance *_instance, uint32_t tx_id);

/**
 * @brief transmit mesg through CAN device,通过can实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 * 
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 * 
 * @param timeout 超时时间,单位为ms;后续改为us,获得更精确的控制
 * @param _instance* can instance owned by module
 */
uint8_t CANTransmit(CANInstance *_instance,float timeout);

/**
 * @brief 等待当前CAN外设的发送邮箱全部完成
 * @param _instance CAN实例
 * @param timeout 超时时间(ms)
 * @return 成功返回1，超时或参数错误返回0
 */
uint8_t CANWaitForTxComplete(CANInstance *_instance, float timeout);

#endif
