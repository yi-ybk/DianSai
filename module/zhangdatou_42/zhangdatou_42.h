/**
 * @file    zhangdatou_42.h
 * @brief   张大头ZDT_X42S闭环步进电机驱动头文件
 * @details 基于Classic CAN扩展帧协议封装电机控制、状态查询和反馈解析接口。
 */
#pragma once

#include "bsp_can.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ZDT42_DEFAULT_CHECKSUM       0x6BU
#define ZDT42_DEFAULT_TX_TIMEOUT_MS  2.0f
#define ZDT42_MAX_SPEED_RPM          5000.0f
#define ZDT42_POSITION_COUNTS_PER_REV 65536.0f

/** @brief 位置运动参考类型 */
typedef enum
{
    ZDT42_POSITION_RELATIVE_TARGET = 0, /**< 相对上一目标位置 */
    ZDT42_POSITION_ABSOLUTE,            /**< 绝对位置 */
    ZDT42_POSITION_RELATIVE_CURRENT,    /**< 相对当前实时位置 */
} Zdt42PositionMode_t;

/** @brief 回零模式 */
typedef enum
{
    ZDT42_HOME_NEAREST = 0,      /**< 单圈就近回零 */
    ZDT42_HOME_DIRECTION,        /**< 单圈方向回零 */
    ZDT42_HOME_COLLISION,        /**< 多圈无限位碰撞回零 */
    ZDT42_HOME_LIMIT_SWITCH,     /**< 多圈限位开关回零 */
} Zdt42HomeMode_t;

/** @brief 常用系统参数功能码 */
typedef enum
{
    ZDT42_PARAM_BUS_VOLTAGE     = 0x24,
    ZDT42_PARAM_BUS_CURRENT     = 0x26,
    ZDT42_PARAM_PHASE_CURRENT   = 0x27,
    ZDT42_PARAM_ENCODER_RAW     = 0x29,
    ZDT42_PARAM_REALTIME_PULSE  = 0x30,
    ZDT42_PARAM_ENCODER_VALUE   = 0x31,
    ZDT42_PARAM_INPUT_PULSE     = 0x32,
    ZDT42_PARAM_TARGET_POSITION = 0x33,
    ZDT42_PARAM_SET_POSITION    = 0x34,
    ZDT42_PARAM_SPEED           = 0x35,
    ZDT42_PARAM_POSITION        = 0x36,
    ZDT42_PARAM_POSITION_ERROR  = 0x37,
    ZDT42_PARAM_TEMPERATURE     = 0x39,
    ZDT42_PARAM_STATUS          = 0x3A,
    ZDT42_PARAM_HOME_STATUS     = 0x3B,
} Zdt42SystemParameter_t;

/** @brief 电机初始化配置 */
typedef struct
{
    MCAN_Regs *mcan;                  /**< MSPM0 MCAN外设寄存器基地址 */
    IRQn_Type irq_number;             /**< MCAN对应的NVIC中断号 */
    uint32_t tx_buffer_index;         /**< SysConfig配置的专用Tx Buffer索引 */
    uint32_t rx_fifo_number;          /**< 接收FIFO编号 */
    uint8_t motor_id;                 /**< 电机地址，范围1~255 */
    uint8_t checksum;                 /**< 电机固定校验字节，0使用默认0x6B */
    float tx_timeout_ms;              /**< 单个CAN帧发送超时时间(ms) */
    void *id;                         /**< 用户自定义标识 */
} Zdt42InitConfig_t;

/** @brief 电机反馈和通信状态 */
typedef struct
{
    float speed_rpm;                /**< 实时有符号转速(RPM) */
    float position_deg;             /**< 实时有符号多圈位置(deg) */
    uint8_t last_function;          /**< 最近反馈功能码 */
    uint8_t last_response;          /**< 最近命令状态或返回数据首字节 */
    uint8_t last_packet_index;      /**< 最近反馈分包编号 */
    uint8_t raw_data[8];            /**< 最近一帧原始数据 */
    uint8_t raw_length;             /**< 最近一帧数据长度 */
    bool command_received;          /**< 最近反馈是否表示命令已接收(0x02) */
    bool motion_complete;           /**< 最近反馈是否表示动作完成(0x9F) */
    uint32_t rx_frame_count;        /**< 接收且校验成功的帧数 */
    uint32_t tx_frame_count;        /**< 发送成功的CAN帧数 */
    uint32_t checksum_error_count;  /**< 校验错误帧数 */
    uint32_t tx_error_count;        /**< 发送失败帧数 */
    uint32_t last_rx_tick;          /**< 最近有效反馈的HAL毫秒时间戳 */
    uint32_t position_rx_tick;      /**< 最近位置反馈的HAL毫秒时间戳 */
} Zdt42Data_t;

typedef struct Zdt42 Zdt42_t;

/** @brief 电机反馈回调，运行于CAN接收中断上下文 */
typedef void (*Zdt42FeedbackCallback_t)(Zdt42_t *motor,
                                        const Zdt42Data_t *data,
                                        void *context);

/** @brief 张大头42步进电机对象 */
struct Zdt42
{
    CANInstance *can;                           /**< 底层CAN实例 */
    bool initialized;                           /**< 初始化标识 */
    Zdt42InitConfig_t init_config;              /**< 初始化配置缓存 */
    Zdt42Data_t data;                           /**< 电机反馈数据 */
    Zdt42FeedbackCallback_t feedback_callback;  /**< 可选反馈回调 */
    void *feedback_context;                     /**< 反馈回调上下文 */

    bool (*init)(Zdt42_t *motor, const Zdt42InitConfig_t *config);
    bool (*enable)(Zdt42_t *motor, bool enabled, bool sync);
    bool (*set_speed)(Zdt42_t *motor, float speed_rpm, uint8_t acceleration, bool sync);
    bool (*move_position)(Zdt42_t *motor, int32_t pulse, uint16_t speed_rpm,
                          uint8_t acceleration, Zdt42PositionMode_t mode, bool sync);
    bool (*set_quick_position)(Zdt42_t *motor, uint16_t speed_rpm,
                               uint8_t acceleration, Zdt42PositionMode_t mode, bool sync);
    bool (*move_quick)(Zdt42_t *motor, int32_t pulse);
    bool (*stop)(Zdt42_t *motor, bool sync);
    bool (*home)(Zdt42_t *motor, Zdt42HomeMode_t mode, bool sync);
    bool (*interrupt_home)(Zdt42_t *motor);
    bool (*set_origin)(Zdt42_t *motor, bool save);
    bool (*reset_position)(Zdt42_t *motor);
    bool (*clear_fault)(Zdt42_t *motor);
    bool (*trigger_sync)(Zdt42_t *motor);
    bool (*read_parameter)(Zdt42_t *motor, Zdt42SystemParameter_t parameter);
    bool (*set_auto_return)(Zdt42_t *motor, Zdt42SystemParameter_t parameter,
                            uint16_t period_ms);
    bool (*send_raw)(Zdt42_t *motor, uint8_t motor_id,
                     const uint8_t *command, uint16_t length);
    void (*get_data)(Zdt42_t *motor, Zdt42Data_t *data);
    void (*set_callback)(Zdt42_t *motor, Zdt42FeedbackCallback_t callback, void *context);
};

bool Zdt42Init(Zdt42_t *motor, const Zdt42InitConfig_t *config);
bool Zdt42Enable(Zdt42_t *motor, bool enabled, bool sync);
bool Zdt42SetSpeed(Zdt42_t *motor, float speed_rpm, uint8_t acceleration, bool sync);
bool Zdt42MovePosition(Zdt42_t *motor, int32_t pulse, uint16_t speed_rpm,
                       uint8_t acceleration, Zdt42PositionMode_t mode, bool sync);
bool Zdt42SetQuickPosition(Zdt42_t *motor, uint16_t speed_rpm,
                           uint8_t acceleration, Zdt42PositionMode_t mode, bool sync);
bool Zdt42MoveQuick(Zdt42_t *motor, int32_t pulse);
bool Zdt42Stop(Zdt42_t *motor, bool sync);
bool Zdt42Home(Zdt42_t *motor, Zdt42HomeMode_t mode, bool sync);
bool Zdt42InterruptHome(Zdt42_t *motor);
bool Zdt42SetOrigin(Zdt42_t *motor, bool save);
bool Zdt42ResetPosition(Zdt42_t *motor);
bool Zdt42ClearFault(Zdt42_t *motor);
bool Zdt42TriggerSync(Zdt42_t *motor);
bool Zdt42ReadParameter(Zdt42_t *motor, Zdt42SystemParameter_t parameter);
bool Zdt42SetAutoReturn(Zdt42_t *motor, Zdt42SystemParameter_t parameter, uint16_t period_ms);
bool Zdt42SendRaw(Zdt42_t *motor, uint8_t motor_id,
                  const uint8_t *command, uint16_t length);
void Zdt42GetData(Zdt42_t *motor, Zdt42Data_t *data);
void Zdt42SetCallback(Zdt42_t *motor, Zdt42FeedbackCallback_t callback, void *context);

#define ZDT42_INIT_CONFIG_DEFAULT(_mcan, _motor_id)     \
    {                                                   \
        .mcan          = (_mcan),                       \
        .irq_number    = CANFD0_INT_IRQn,               \
        .tx_buffer_index = 0U,                          \
        .rx_fifo_number  = DL_MCAN_RX_FIFO_NUM_0,       \
        .motor_id      = (_motor_id),                   \
        .checksum      = ZDT42_DEFAULT_CHECKSUM,        \
        .tx_timeout_ms = ZDT42_DEFAULT_TX_TIMEOUT_MS,   \
        .id            = NULL,                          \
    }

#define ZDT42_OBJECT_DEFAULT                            \
    .init               = Zdt42Init,                    \
    .enable             = Zdt42Enable,                  \
    .set_speed          = Zdt42SetSpeed,                \
    .move_position      = Zdt42MovePosition,            \
    .set_quick_position = Zdt42SetQuickPosition,        \
    .move_quick         = Zdt42MoveQuick,               \
    .stop               = Zdt42Stop,                    \
    .home               = Zdt42Home,                    \
    .interrupt_home     = Zdt42InterruptHome,           \
    .set_origin         = Zdt42SetOrigin,               \
    .reset_position     = Zdt42ResetPosition,           \
    .clear_fault        = Zdt42ClearFault,              \
    .trigger_sync       = Zdt42TriggerSync,             \
    .read_parameter     = Zdt42ReadParameter,           \
    .set_auto_return    = Zdt42SetAutoReturn,           \
    .send_raw           = Zdt42SendRaw,                 \
    .get_data           = Zdt42GetData,                 \
    .set_callback       = Zdt42SetCallback
