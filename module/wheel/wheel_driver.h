/**
 * @file    wheel_driver.h
 * @brief   驱动轮模块头文件
 * @details 基于减速电机、编码器和可选PID封装轮级控制接口，提供速度控制、里程计算和对象式访问接口。
 */
#pragma once

#include "motor_driver.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 驱动轮控制模式 */
typedef enum
{
    WHEEL_CONTROL_OPEN_LOOP = 0, /**< 开环输出控制 */
    WHEEL_CONTROL_SPEED,         /**< 速度闭环控制 */
} WheelControlMode_t;

/** @brief 驱动轮初始化配置 */
typedef struct
{
    Motor_t *motor;                 /**< 绑定的减速电机对象 */
    Pid_t *speed_pid;               /**< 绑定的速度PID对象 */
    bool use_speed_pid;             /**< 是否启用速度PID */

    float radius_m;                 /**< 轮半径(m) */
    float encoder_to_wheel_ratio;   /**< 编码器轴转一圈对应的轮子圈数换算分母，减速箱前编码器通常填减速比 */
    float max_linear_speed_mps;     /**< 开环线速度映射的最大线速度(m/s) */

    float output_min;               /**< 电机输出下限 */
    float output_max;               /**< 电机输出上限 */
    bool reversed;                  /**< 是否反转轮子逻辑方向 */
    bool auto_start;                /**< 初始化后是否自动启动电机 */
} WheelInitConfig_t;

/** @brief 驱动轮运行数据 */
typedef struct
{
    bool enabled;                         /**< 当前是否使能 */
    WheelControlMode_t control_mode;      /**< 当前控制模式 */
    MotorDirection_t direction;           /**< 当前轮子方向 */

    float target_linear_speed_mps;        /**< 目标线速度(m/s) */
    float target_angular_speed_radps;     /**< 目标角速度(rad/s) */
    float linear_speed_mps;               /**< 实际线速度(m/s) */
    float angular_speed_radps;            /**< 实际角速度(rad/s) */
    float wheel_speed_rps;                /**< 轮子转速(rev/s) */

    float distance_m;                     /**< 累计行驶距离(m) */
    float angle_rad;                      /**< 累计轮子转角(rad) */

    uint32_t update_count;                /**< 更新次数 */
} WheelData_t;

typedef struct Wheel Wheel_t;

/** @brief 驱动轮对象 */
struct Wheel
{
    Motor_t *motor;                  /**< 绑定的减速电机对象 */
    Pid_t *speed_pid;                /**< 绑定的速度PID对象 */

    bool initialized;                /**< 初始化标记 */
    WheelInitConfig_t init_config;   /**< 初始化配置缓存 */
    WheelData_t data;                /**< 当前运行数据 */

    bool (*init)(Wheel_t *wheel, const WheelInitConfig_t *config);
    void (*start)(Wheel_t *wheel);
    void (*stop)(Wheel_t *wheel);
    void (*set_linear_speed)(Wheel_t *wheel, float speed_mps);
    void (*set_angular_speed)(Wheel_t *wheel, float speed_radps);
    void (*update)(Wheel_t *wheel, float dt_s);
    void (*reset_odometry)(Wheel_t *wheel);
    void (*get_data)(Wheel_t *wheel, WheelData_t *data);
};

/** @brief 初始化驱动轮对象 */
bool WheelInit(Wheel_t *wheel, const WheelInitConfig_t *config);
/** @brief 启动驱动轮 */
void WheelStart(Wheel_t *wheel);
/** @brief 停止驱动轮 */
void WheelStop(Wheel_t *wheel);
/** @brief 设置目标线速度 */
void WheelSetLinearSpeed(Wheel_t *wheel, float speed_mps);
/** @brief 设置目标角速度 */
void WheelSetAngularSpeed(Wheel_t *wheel, float speed_radps);
/** @brief 更新驱动轮状态和闭环输出 */
void WheelUpdate(Wheel_t *wheel, float dt_s);
/** @brief 复位驱动轮里程 */
void WheelResetOdometry(Wheel_t *wheel);
/** @brief 获取驱动轮运行数据 */
void WheelGetData(Wheel_t *wheel, WheelData_t *data);

#define WHEEL_OBJECT_DEFAULT                       \
        .init              = WheelInit,            \
        .start             = WheelStart,           \
        .stop              = WheelStop,            \
        .set_linear_speed  = WheelSetLinearSpeed,  \
        .set_angular_speed = WheelSetAngularSpeed, \
        .update            = WheelUpdate,          \
        .reset_odometry    = WheelResetOdometry,   \
        .get_data          = WheelGetData
