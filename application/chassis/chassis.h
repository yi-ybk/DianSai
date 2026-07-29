/**
 * @file    chassis.h
 * @brief   底盘组件头文件
 * @details 聚合可变数量的驱动轮对象并可选绑定IMU，仅提供底盘级控制和状态访问接口。
 */
#pragma once

#include "imu_driver.h"
#include "wheel_driver.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 单个驱动轮运动学配置 */
typedef struct
{
    float forward_coefficient; /**< 前进速度映射系数，常规驱动轮填1 */
    float turn_coefficient_m;  /**< 转向角速度映射系数(m)，左轮通常为负、右轮通常为正 */
} ChassisWheelKinematics_t;

/** @brief 底盘初始化配置 */
typedef struct
{
    Wheel_t *const *wheels;                           /**< 驱动轮对象指针数组，数组生命周期需覆盖底盘对象 */
    const ChassisWheelKinematics_t *wheel_kinematics; /**< 驱动轮运动学配置数组，顺序与wheels一致 */
    uint8_t wheel_count;                              /**< 驱动轮数量 */
    Imu_t *imu;                                       /**< 底盘内部绑定的IMU对象，允许为NULL */
    bool auto_start;                                  /**< 初始化后是否自动启动底盘 */
} ChassisInitConfig_t;

/** @brief 底盘运行数据 */
typedef struct
{
    bool enabled;                    /**< 底盘是否处于使能状态 */
    uint8_t wheel_count;             /**< 底盘驱动轮数量 */
    float target_forward_speed_mps;  /**< 请求的底盘前进速度(m/s)，负值表示后退 */
    float target_turn_speed_radps;   /**< 请求的底盘转向角速度(rad/s)，正值表示逆时针 */
    float command_forward_speed_mps; /**< 限速后实际下发的底盘前进速度(m/s) */
    float command_turn_speed_radps;  /**< 限速后实际下发的底盘转向角速度(rad/s) */
    float forward_speed_mps;         /**< 根据驱动轮反馈估算的底盘前进速度(m/s) */
    float turn_speed_radps;          /**< 根据驱动轮反馈估算的底盘转向角速度(rad/s) */
    float distance_m;                /**< 底盘中心累计前进距离(m)，后退时为负 */
    float turn_angle_rad;            /**< 根据驱动轮里程估算的底盘累计转角(rad) */
    float wheel_speed_scale;         /**< 驱动轮超速时的等比例缩放系数 */
    uint32_t update_count;           /**< 底盘更新次数 */
} ChassisData_t;

typedef struct Chassis Chassis_t;

/** @brief 底盘对象 */
struct Chassis
{
    Wheel_t *const *wheels;                           /**< 内部绑定的驱动轮对象指针数组 */
    const ChassisWheelKinematics_t *wheel_kinematics; /**< 内部使用的驱动轮运动学配置数组 */
    Imu_t *imu;                                       /**< 内部绑定的IMU对象 */

    bool initialized;                /**< 初始化标记 */
    ChassisInitConfig_t init_config; /**< 初始化配置缓存 */
    ChassisData_t data;              /**< 当前底盘运行数据 */

    bool (*init)(Chassis_t *chassis, const ChassisInitConfig_t *config);
    void (*start)(Chassis_t *chassis);
    void (*stop)(Chassis_t *chassis);
    bool (*set_velocity)(Chassis_t *chassis, float forward_speed_mps, float turn_speed_radps);
    bool (*set_forward_speed)(Chassis_t *chassis, float forward_speed_mps);
    bool (*set_turn_speed)(Chassis_t *chassis, float turn_speed_radps);
    void (*update)(Chassis_t *chassis, float dt_s);
    void (*reset_odometry)(Chassis_t *chassis);
    void (*get_data)(Chassis_t *chassis, ChassisData_t *data);
};

/** @brief 初始化底盘对象 */
bool ChassisInit(Chassis_t *chassis, const ChassisInitConfig_t *config);
/** @brief 启动底盘 */
void ChassisStart(Chassis_t *chassis);
/** @brief 停止底盘 */
void ChassisStop(Chassis_t *chassis);
/** @brief 设置底盘前进和转向速度 */
bool ChassisSetVelocity(Chassis_t *chassis, float forward_speed_mps, float turn_speed_radps);
/** @brief 设置底盘前进速度并保持当前转向速度 */
bool ChassisSetForwardSpeed(Chassis_t *chassis, float forward_speed_mps);
/** @brief 设置底盘转向速度并保持当前前进速度 */
bool ChassisSetTurnSpeed(Chassis_t *chassis, float turn_speed_radps);
/** @brief 更新底盘控制和运行状态 */
void ChassisUpdate(Chassis_t *chassis, float dt_s);
/** @brief 复位底盘里程 */
void ChassisResetOdometry(Chassis_t *chassis);
/** @brief 获取底盘运行数据 */
void ChassisGetData(Chassis_t *chassis, ChassisData_t *data);

#define CHASSIS_OBJECT_DEFAULT                            \
        .init              = ChassisInit,                \
        .start             = ChassisStart,               \
        .stop              = ChassisStop,                \
        .set_velocity      = ChassisSetVelocity,         \
        .set_forward_speed = ChassisSetForwardSpeed,     \
        .set_turn_speed    = ChassisSetTurnSpeed,        \
        .update            = ChassisUpdate,              \
        .reset_odometry    = ChassisResetOdometry,       \
        .get_data          = ChassisGetData
