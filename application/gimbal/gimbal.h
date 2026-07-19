/**
 * @file    gimbal.h
 * @brief   二维云台组件头文件
 * @details 聚合水平轴和俯仰轴闭环步进电机，提供云台级角度、角速度、回零和状态接口。
 */
#pragma once

#include "zhangdatou_42.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 云台控制模式 */
typedef enum
{
    GIMBAL_CONTROL_IDLE = 0, /**< 空闲或停止 */
    GIMBAL_CONTROL_POSITION, /**< 角度控制 */
    GIMBAL_CONTROL_VELOCITY, /**< 角速度控制 */
    GIMBAL_CONTROL_HOMING,   /**< 回零控制 */
} GimbalControlMode_t;

/** @brief 单个云台轴配置 */
typedef struct
{
    Zdt42_t *motor;               /**< 轴绑定的张大头42电机对象 */
    float pulses_per_motor_rev;   /**< 电机每圈位置命令脉冲数 */
    float motor_to_axis_ratio;    /**< 电机转数与云台轴转数之比 */
    float min_angle_deg;          /**< 云台轴最小机械角度(deg) */
    float max_angle_deg;          /**< 云台轴最大机械角度(deg) */
    float position_offset_deg;    /**< 电机位置为0时对应的云台轴角度(deg) */
    bool reversed;                /**< 电机正方向是否与云台轴正方向相反 */
    Zdt42HomeMode_t home_mode;    /**< 该轴使用的回零模式 */
} GimbalAxisConfig_t;

/** @brief 二维云台初始化配置 */
typedef struct
{
    GimbalAxisConfig_t yaw;       /**< 水平轴配置 */
    GimbalAxisConfig_t pitch;     /**< 俯仰轴配置 */
    uint16_t position_speed_rpm;  /**< 位置控制默认电机转速(RPM) */
    uint8_t acceleration;         /**< 默认加速度，0表示直接启动 */
    uint16_t feedback_period_ms;  /**< 电机位置和速度自动反馈周期，0表示不启用 */
    uint16_t feedback_timeout_ms; /**< 反馈通信超时时间，0表示不检测 */
    bool auto_enable;             /**< 初始化后是否自动使能两个轴 */
} GimbalInitConfig_t;

/** @brief 二维云台运行数据 */
typedef struct
{
    bool enabled;                    /**< 两个轴是否已使能 */
    bool communication_ok;           /**< 两个轴反馈是否均未超时 */
    bool target_limited;             /**< 最近角度命令是否触发机械限位 */
    bool motion_complete;            /**< 两个轴最近反馈是否均为动作完成 */
    GimbalControlMode_t control_mode; /**< 当前控制模式 */
    float target_yaw_deg;            /**< 限位后目标水平角度(deg) */
    float target_pitch_deg;          /**< 限位后目标俯仰角度(deg) */
    float target_yaw_speed_dps;      /**< 目标水平角速度(deg/s) */
    float target_pitch_speed_dps;    /**< 目标俯仰角速度(deg/s) */
    float yaw_deg;                   /**< 当前水平角度(deg) */
    float pitch_deg;                 /**< 当前俯仰角度(deg) */
    float yaw_speed_dps;             /**< 当前水平角速度(deg/s) */
    float pitch_speed_dps;           /**< 当前俯仰角速度(deg/s) */
    uint32_t update_count;           /**< 状态更新次数 */
} GimbalData_t;

typedef struct Gimbal Gimbal_t;

/** @brief 二维云台对象 */
struct Gimbal
{
    Zdt42_t *yaw_motor;             /**< 内部绑定的水平轴电机 */
    Zdt42_t *pitch_motor;           /**< 内部绑定的俯仰轴电机 */
    bool initialized;               /**< 初始化标识 */
    uint32_t last_feedback_request_tick; /**< 最近一次主动请求位置反馈的时间 */
    GimbalInitConfig_t init_config; /**< 初始化配置缓存 */
    GimbalData_t data;              /**< 当前运行数据 */

    bool (*init)(Gimbal_t *gimbal, const GimbalInitConfig_t *config);
    bool (*enable)(Gimbal_t *gimbal, bool enabled);
    bool (*set_angle)(Gimbal_t *gimbal, float yaw_deg, float pitch_deg);
    bool (*set_yaw_angle)(Gimbal_t *gimbal, float yaw_deg);
    bool (*set_pitch_angle)(Gimbal_t *gimbal, float pitch_deg);
    bool (*set_angular_velocity)(Gimbal_t *gimbal,
                                 float yaw_speed_dps,
                                 float pitch_speed_dps);
    bool (*stop)(Gimbal_t *gimbal);
    bool (*home)(Gimbal_t *gimbal);
    bool (*zero)(Gimbal_t *gimbal);
    void (*update)(Gimbal_t *gimbal);
    void (*get_data)(Gimbal_t *gimbal, GimbalData_t *data);
};

bool GimbalInit(Gimbal_t *gimbal, const GimbalInitConfig_t *config);
bool GimbalEnable(Gimbal_t *gimbal, bool enabled);
bool GimbalSetAngle(Gimbal_t *gimbal, float yaw_deg, float pitch_deg);
bool GimbalSetYawAngle(Gimbal_t *gimbal, float yaw_deg);
bool GimbalSetPitchAngle(Gimbal_t *gimbal, float pitch_deg);
bool GimbalSetAngularVelocity(Gimbal_t *gimbal,
                              float yaw_speed_dps,
                              float pitch_speed_dps);
bool GimbalStop(Gimbal_t *gimbal);
bool GimbalHome(Gimbal_t *gimbal);
bool GimbalZero(Gimbal_t *gimbal);
void GimbalUpdate(Gimbal_t *gimbal);
void GimbalGetData(Gimbal_t *gimbal, GimbalData_t *data);

#define GIMBAL_OBJECT_DEFAULT                         \
    .init                 = GimbalInit,               \
    .enable               = GimbalEnable,             \
    .set_angle            = GimbalSetAngle,           \
    .set_yaw_angle        = GimbalSetYawAngle,        \
    .set_pitch_angle      = GimbalSetPitchAngle,      \
    .set_angular_velocity = GimbalSetAngularVelocity, \
    .stop                 = GimbalStop,               \
    .home                 = GimbalHome,               \
    .zero                 = GimbalZero,               \
    .update               = GimbalUpdate,             \
    .get_data             = GimbalGetData
