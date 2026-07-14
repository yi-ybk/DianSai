/**
 * @file    motor_driver.h
 * @brief   电机驱动模块头文件
 * @details 封装减速电机与舵机的统一控制接口，并可选集成编码器反馈。
 */
#pragma once

#include "bsp_pwm.h"
#include "gpio.h"
#include "encoder.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 电机类型 */
typedef enum
{
    MOTOR_TYPE_REDUCTION = 0, /**< 减速电机 */
    MOTOR_TYPE_SERVO,         /**< 舵机 */
} MotorType_t;

/** @brief 电机方向 */
typedef enum
{
    MOTOR_DIR_STOP = 0,    /**< 停止 */
    MOTOR_DIR_FORWARD,     /**< 正转 */
    MOTOR_DIR_REVERSE,     /**< 反转 */
} MotorDirection_t;

/** @brief PWM输出配置 */
typedef struct
{
    TIM_HandleTypeDef *htim; /**< PWM对应定时器句柄 */
    uint32_t channel;        /**< PWM通道 */
    float period;            /**< PWM周期(s) */
    float init_duty;         /**< 初始占空比 */
} MotorPwmConfig_t;

/** @brief GPIO输出配置 */
typedef struct
{
    GPIO_TypeDef *GPIOx; /**< GPIO端口 */
    uint16_t GPIO_Pin;   /**< GPIO引脚 */
} MotorGpioConfig_t;

/** @brief TB6612方向控制配置 */
typedef struct
{
    MotorGpioConfig_t in1;     /**< IN1方向引脚 */
    MotorGpioConfig_t in2;     /**< IN2方向引脚 */
    MotorGpioConfig_t standby; /**< STBY使能引脚 */
    bool use_standby;          /**< 是否使用STBY引脚 */
    bool brake_on_stop;        /**< 停止时是否短刹 */
    bool reversed;             /**< 是否交换正反转方向 */
} MotorTb6612Config_t;

/** @brief 电机初始化配置 */
typedef struct
{
    MotorType_t type;             /**< 电机类型 */
    MotorPwmConfig_t pwm;         /**< 主PWM配置 */
    MotorPwmConfig_t reverse_pwm; /**< 反向PWM配置 */
    bool use_reverse_pwm;         /**< 是否启用双PWM方向控制 */
    bool use_tb6612;              /**< 是否启用TB6612方向控制 */
    MotorTb6612Config_t tb6612;   /**< TB6612方向控制配置 */

    bool use_encoder;             /**< 是否启用编码器反馈 */
    Encoder_t *encoder;           /**< 绑定的编码器对象 */

    float min_duty;               /**< 最小占空比限制 */
    float max_duty;               /**< 最大占空比限制 */

    float servo_min_angle;        /**< 舵机最小角度 */
    float servo_max_angle;        /**< 舵机最大角度 */
    float servo_min_pulse_us;     /**< 舵机最小脉宽(us) */
    float servo_max_pulse_us;     /**< 舵机最大脉宽(us) */

    float init_output;            /**< 初始输出值 */
} MotorInitConfig_t;

/** @brief 电机运行数据 */
typedef struct
{
    MotorType_t type;           /**< 电机类型 */
    bool enabled;               /**< 当前是否使能 */
    MotorDirection_t direction; /**< 当前驱动方向 */
    float output;               /**< 当前输出命令 */
    float angle;                /**< 当前目标角度（舵机模式） */
    float speed_rps;            /**< 当前反馈转速(rev/s)，未绑定编码器时为0 */
} MotorData_t;

typedef struct Motor Motor_t;

/** @brief 电机对象 */
struct Motor
{
    PWMInstance *pwm;             /**< 主PWM对象 */
    PWMInstance *reverse_pwm;     /**< 反向PWM对象 */
    Encoder_t *encoder;           /**< 编码器对象 */

    bool initialized;              /**< 初始化标记 */
    MotorInitConfig_t init_config; /**< 初始化配置缓存 */
    MotorData_t data;              /**< 当前数据 */

    bool (*init)(Motor_t *motor, const MotorInitConfig_t *config);
    void (*start)(Motor_t *motor);
    void (*stop)(Motor_t *motor);
    void (*set_output)(Motor_t *motor, float output);
    void (*set_speed)(Motor_t *motor, float speed);
    void (*set_angle)(Motor_t *motor, float angle);
    void (*update)(Motor_t *motor, float dt_s);
    void (*get_data)(Motor_t *motor, MotorData_t *data);
};

/** @brief 初始化电机对象 */
bool MotorInit(Motor_t *motor, const MotorInitConfig_t *config);
/** @brief 启动电机 */
void MotorStart(Motor_t *motor);
/** @brief 停止电机 */
void MotorStop(Motor_t *motor);
/** @brief 设置统一输出 */
void MotorSetOutput(Motor_t *motor, float output);
/** @brief 设置减速电机速度 */
void MotorSetSpeed(Motor_t *motor, float speed);
/** @brief 设置舵机角度 */
void MotorSetAngle(Motor_t *motor, float angle);
/** @brief 刷新电机和编码器数据 */
void MotorUpdate(Motor_t *motor, float dt_s);
/** @brief 获取电机数据 */
void MotorGetData(Motor_t *motor, MotorData_t *data);

#define MOTOR_OBJECT_DEFAULT          \
        .init       = MotorInit,      \
        .start      = MotorStart,     \
        .stop       = MotorStop,      \
        .set_output = MotorSetOutput, \
        .set_speed  = MotorSetSpeed,  \
        .set_angle  = MotorSetAngle,  \
        .update     = MotorUpdate,    \
        .get_data   = MotorGetData
