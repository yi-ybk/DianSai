/**
 * @file    pid.h
 * @brief   PID控制组件头文件
 * @details 提供通用位置式PID控制器，支持输出限幅、积分限幅、误差死区和对象式访问接口。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** @brief PID初始化配置 */
typedef struct
{
    float kp;                         /**< 比例系数 */
    float ki;                         /**< 积分系数 */
    float kd;                         /**< 微分系数 */

    bool enable_output_limit;         /**< 是否启用输出限幅 */
    float output_min;                 /**< 输出最小值 */
    float output_max;                 /**< 输出最大值 */

    bool enable_integral_limit;       /**< 是否启用积分项限幅 */
    float integral_min;               /**< 积分输出最小值 */
    float integral_max;               /**< 积分输出最大值 */

    float deadband;                   /**< 误差死区，误差绝对值小于该值时视为0 */
    bool derivative_on_measurement;   /**< 是否对测量值做微分，启用后可减小目标阶跃引起的微分冲击 */
    bool reset_integral_on_deadband;  /**< 进入死区时是否清除积分 */
} PidInitConfig_t;

/** @brief PID运行数据 */
typedef struct
{
    float target;          /**< 目标值 */
    float feedback;        /**< 反馈值 */
    float error;           /**< 当前误差 */
    float last_error;      /**< 上一次误差 */
    float last_feedback;   /**< 上一次反馈值 */
    float integral;        /**< 积分累加量 */
    float derivative;      /**< 微分量 */

    float p_out;           /**< 比例输出 */
    float i_out;           /**< 积分输出 */
    float d_out;           /**< 微分输出 */
    float output;          /**< 当前输出 */
    float last_output;     /**< 上一次输出 */
    float dt_s;            /**< 本次计算周期(s) */

    bool in_deadband;      /**< 当前是否处于误差死区 */
    uint32_t update_count; /**< 计算次数 */
} PidData_t;

typedef struct Pid Pid_t;

/** @brief PID控制器对象 */
struct Pid
{
    bool initialized;             /**< 初始化标记 */
    PidInitConfig_t init_config;  /**< 初始化配置缓存 */
    PidData_t data;               /**< 当前运行数据 */

    bool (*init)(Pid_t *pid, const PidInitConfig_t *config);
    float (*calculate)(Pid_t *pid, float feedback, float target, float dt_s);
    void (*reset)(Pid_t *pid);
    void (*set_target)(Pid_t *pid, float target);
    void (*set_param)(Pid_t *pid, float kp, float ki, float kd);
    void (*set_output_limit)(Pid_t *pid, bool enable, float output_min, float output_max);
    void (*set_integral_limit)(Pid_t *pid, bool enable, float integral_min, float integral_max);
    void (*get_data)(Pid_t *pid, PidData_t *data);
};

/** @brief 初始化PID对象 */
bool PidInit(Pid_t *pid, const PidInitConfig_t *config);
/** @brief 计算PID输出 */
float PidCalculate(Pid_t *pid, float feedback, float target, float dt_s);
/** @brief 复位PID运行数据 */
void PidReset(Pid_t *pid);
/** @brief 设置PID目标值 */
void PidSetTarget(Pid_t *pid, float target);
/** @brief 设置PID参数 */
void PidSetParam(Pid_t *pid, float kp, float ki, float kd);
/** @brief 设置PID输出限幅 */
void PidSetOutputLimit(Pid_t *pid, bool enable, float output_min, float output_max);
/** @brief 设置PID积分项限幅 */
void PidSetIntegralLimit(Pid_t *pid, bool enable, float integral_min, float integral_max);
/** @brief 获取PID运行数据 */
void PidGetData(Pid_t *pid, PidData_t *data);

#define PID_OBJECT_DEFAULT                                      \
        .init               = PidInit,                          \
        .calculate          = PidCalculate,                     \
        .reset              = PidReset,                         \
        .set_target         = PidSetTarget,                     \
        .set_param          = PidSetParam,                      \
        .set_output_limit   = PidSetOutputLimit,                \
        .set_integral_limit = PidSetIntegralLimit,              \
        .get_data           = PidGetData
