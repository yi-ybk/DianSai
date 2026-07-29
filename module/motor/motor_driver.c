/**
 * @file    motor_driver.c
 * @brief   电机驱动模块实现
 * @details 封装PWM输出、舵机脉宽换算、编码器同步和对象式方法绑定。
 */
#include "motor_driver.h"
#include "user_lib_math.h"
#include <math.h>
#include <string.h>

#define MOTOR_DEFAULT_MIN_DUTY 0.0f              /**< 默认最小占空比 */
#define MOTOR_DEFAULT_MAX_DUTY 1.0f              /**< 默认最大占空比 */
#define MOTOR_DEFAULT_SERVO_MIN_ANGLE 0.0f       /**< 舵机默认最小角度 */
#define MOTOR_DEFAULT_SERVO_MAX_ANGLE 180.0f     /**< 舵机默认最大角度 */
#define MOTOR_DEFAULT_SERVO_MIN_PULSE_US 500.0f  /**< 舵机默认最小脉宽(us) */
#define MOTOR_DEFAULT_SERVO_MAX_PULSE_US 2500.0f /**< 舵机默认最大脉宽(us) */
#define MOTOR_US_PER_SECOND 1000000.0f           /**< 1秒对应的微秒数 */

static void MotorBindMethods(Motor_t *motor);
static bool MotorPwmConfigIsValid(const MotorPwmConfig_t *config);
static bool MotorGpioConfigIsValid(const MotorGpioConfig_t *config);
static bool MotorTb6612ConfigIsValid(const MotorInitConfig_t *config);
static bool MotorEncoderConfigIsValid(const MotorInitConfig_t *config);
static void MotorNormalizeConfig(MotorInitConfig_t *config);
static bool MotorRegisterPwm(Motor_t *motor);
static void MotorSyncEncoderData(Motor_t *motor);
static void MotorWriteGpio(const MotorGpioConfig_t *config, GPIO_PinState state);
static void MotorTb6612SetStandby(Motor_t *motor, bool enable);
static void MotorTb6612ApplyDirection(Motor_t *motor, MotorDirection_t direction);
static void MotorApplyReductionOutput(Motor_t *motor, float speed);
static void MotorApplyServoPulse(Motor_t *motor, float pulse_width_us);

/**
 * @brief   初始化电机对象
 * @details 完成方法绑定、配置归一化、PWM实例注册、可选编码器关联和初始输出下发。
 * @param   motor 电机对象指针
 * @param   config 初始化配置指针
 * @return  bool 成功返回true，失败返回false
 */
bool MotorInit(Motor_t *motor, const MotorInitConfig_t *config)
{
    if ((motor == NULL) || (config == NULL))
        return false;

    MotorBindMethods(motor);
    if (motor->initialized)
        return true;

    motor->init_config = *config;
    MotorNormalizeConfig(&motor->init_config);

    if (!MotorPwmConfigIsValid(&motor->init_config.pwm))
        return false;

    if ((motor->init_config.type == MOTOR_TYPE_REDUCTION) &&
        (!MotorEncoderConfigIsValid(&motor->init_config)))
    {
        return false;
    }

    if ((motor->init_config.type == MOTOR_TYPE_REDUCTION) &&
        (motor->init_config.use_reverse_pwm) &&
        (!MotorPwmConfigIsValid(&motor->init_config.reverse_pwm)))
    {
        return false;
    }

    if ((motor->init_config.type == MOTOR_TYPE_REDUCTION) &&
        (!MotorTb6612ConfigIsValid(&motor->init_config)))
    {
        return false;
    }

    memset(&motor->data, 0, sizeof(motor->data));
    motor->data.type = motor->init_config.type;
    motor->encoder   = motor->init_config.use_encoder ? motor->init_config.encoder : NULL;

    if (!MotorRegisterPwm(motor))
        return false;

    if (motor->init_config.use_tb6612)
    {
        MotorTb6612ApplyDirection(motor, MOTOR_DIR_STOP);
        MotorTb6612SetStandby(motor, false);
    }

    motor->initialized = true;
    MotorStart(motor);
    MotorSetOutput(motor, motor->init_config.init_output);
    MotorSyncEncoderData(motor);

    return true;
}

/**
 * @brief   启动电机输出
 * @details 启动主PWM与可选反向PWM；若绑定编码器且尚未运行，会尝试启动编码器。
 * @param   motor 电机对象指针
 */
void MotorStart(Motor_t *motor)
{
    if ((motor == NULL) || (!motor->initialized))
        return;

    PWMStart(motor->pwm);
    if (motor->reverse_pwm != NULL)
        PWMStart(motor->reverse_pwm);

    if (motor->init_config.use_tb6612)
        MotorTb6612SetStandby(motor, true);

    if ((motor->encoder != NULL) && (!motor->encoder->data.enabled))
        (void)EncoderStart(motor->encoder);

    motor->data.enabled = true;
    MotorSyncEncoderData(motor);
}

/**
 * @brief   停止电机输出
 * @details 将PWM占空比拉为0并停止通道，同时清理电机方向和输出命令。
 * @param   motor 电机对象指针
 */
void MotorStop(Motor_t *motor)
{
    if ((motor == NULL) || (!motor->initialized))
        return;

    PWMSetDutyRatio(motor->pwm, 0.0f);
    PWMStop(motor->pwm);

    if (motor->reverse_pwm != NULL)
    {
        PWMSetDutyRatio(motor->reverse_pwm, 0.0f);
        PWMStop(motor->reverse_pwm);
    }

    if (motor->init_config.use_tb6612)
    {
        MotorTb6612ApplyDirection(motor, MOTOR_DIR_STOP);
        if (!motor->init_config.tb6612.brake_on_stop)
            MotorTb6612SetStandby(motor, false);
    }

    motor->data.enabled   = false;
    motor->data.direction = MOTOR_DIR_STOP;
    motor->data.output    = 0.0f;
    MotorSyncEncoderData(motor);
}

/**
 * @brief   统一设置电机输出
 * @details 减速电机视为速度输入，舵机视为角度输入。
 * @param   motor 电机对象指针
 * @param   output 输出目标值（减速电机为速度比例，舵机为目标角度）
 */
void MotorSetOutput(Motor_t *motor, float output)
{
    if ((motor == NULL) || (!motor->initialized))
        return;

    if (motor->init_config.type == MOTOR_TYPE_SERVO)
        MotorSetAngle(motor, output);
    else
        MotorSetSpeed(motor, output);
}

/**
 * @brief   设置减速电机速度
 * @details 支持单PWM或双PWM反转控制模式，内部会执行限幅和映射。
 * @param   motor 电机对象指针
 * @param   speed 速度控制量（通常范围 0~1 或 -1~1）
 */
void MotorSetSpeed(Motor_t *motor, float speed)
{
    if ((motor == NULL) || (!motor->initialized))
        return;

    if (motor->init_config.type != MOTOR_TYPE_REDUCTION)
        return;

    MotorApplyReductionOutput(motor, speed);
}

/**
 * @brief   设置舵机角度
 * @details 将角度线性映射到配置的脉宽区间，再下发到PWM脉宽接口。
 * @param   motor 电机对象指针
 * @param   angle 目标角度（度）
 */
void MotorSetAngle(Motor_t *motor, float angle)
{
    float pulse_width_us;
    float span_angle;
    float span_pulse;

    if ((motor == NULL) || (!motor->initialized))
        return;

    if (motor->init_config.type != MOTOR_TYPE_SERVO)
        return;

    angle = float_constrain(angle,
                            motor->init_config.servo_min_angle,
                            motor->init_config.servo_max_angle);

    span_angle     = motor->init_config.servo_max_angle    - motor->init_config.servo_min_angle;
    span_pulse     = motor->init_config.servo_max_pulse_us - motor->init_config.servo_min_pulse_us;
    pulse_width_us = motor->init_config.servo_min_pulse_us +
                     ((angle - motor->init_config.servo_min_angle) * span_pulse / span_angle);

    MotorApplyServoPulse(motor, pulse_width_us);
    motor->data.angle = angle;
    motor->data.output = angle;
}

/**
 * @brief   刷新电机运行状态和编码器信息
 * @details 若绑定了编码器对象，会驱动编码器更新并同步结果到motor->data。
 * @param   motor 电机对象指针
 * @param   dt_s 更新周期（秒）
 */
void MotorUpdate(Motor_t *motor, float dt_s)
{
    if ((motor == NULL) || (!motor->initialized))
        return;

    if (motor->encoder != NULL)
        EncoderUpdate(motor->encoder, dt_s);

    MotorSyncEncoderData(motor);
}

/**
 * @brief   获取电机运行数据
 * @details 返回前会先同步一次电机实际转速，保证data内容尽可能新鲜。
 * @param   motor 电机对象指针
 * @param   data 输出结构体指针
 */
void MotorGetData(Motor_t *motor, MotorData_t *data)
{
    if ((motor == NULL) || (data == NULL))
        return;

    MotorSyncEncoderData(motor);
    memcpy(data, &motor->data, sizeof(*data));
}

/**
 * @brief   绑定对象方法
 * @param   motor 电机对象指针
 */
static void MotorBindMethods(Motor_t *motor)
{
    if (motor == NULL)
        return;

    motor->init       = MotorInit;
    motor->start      = MotorStart;
    motor->stop       = MotorStop;
    motor->set_output = MotorSetOutput;
    motor->set_speed  = MotorSetSpeed;
    motor->set_angle  = MotorSetAngle;
    motor->update     = MotorUpdate;
    motor->get_data   = MotorGetData;
}

/**
 * @brief   校验PWM配置是否合法
 * @param   config PWM配置指针
 * @return  bool 合法返回true，否则返回false
 */
static bool MotorPwmConfigIsValid(const MotorPwmConfig_t *config)
{
    if ((config == NULL) || (config->htim == NULL) || (config->period <= 0.0f))
        return false;

    if ((config->channel != TIM_CHANNEL_1) &&
        (config->channel != TIM_CHANNEL_2) &&
        (config->channel != TIM_CHANNEL_3) &&
        (config->channel != TIM_CHANNEL_4))
    {
        return false;
    }

    return true;
}

/**
 * @brief   校验编码器相关配置是否合法
 * @param   config 电机初始化配置指针
 * @return  bool 合法返回true，否则返回false
 */
static bool MotorGpioConfigIsValid(const MotorGpioConfig_t *config)
{
    if ((config == NULL) || (config->GPIOx == NULL) || (config->GPIO_Pin == 0U))
        return false;

    return true;
}

static bool MotorTb6612ConfigIsValid(const MotorInitConfig_t *config)
{
    if ((config == NULL) || (!config->use_tb6612))
        return true;

    if ((!MotorGpioConfigIsValid(&config->tb6612.in1)) ||
        (!MotorGpioConfigIsValid(&config->tb6612.in2)))
    {
        return false;
    }

    if ((config->tb6612.use_standby) &&
        (!MotorGpioConfigIsValid(&config->tb6612.standby)))
    {
        return false;
    }

    return true;
}

static bool MotorEncoderConfigIsValid(const MotorInitConfig_t *config)
{
    if ((config == NULL) || (!config->use_encoder))
        return true;

    if (config->encoder == NULL)
        return false;

    if (!config->encoder->initialized)
        return false;

    return true;
}

/**
 * @brief   归一化初始化配置，补齐默认值并修正非法范围
 * @details 会对占空比、舵机角度/脉宽边界进行修正，并根据电机类型修正编码器开关。
 * @param   config 电机初始化配置指针
 */
static void MotorNormalizeConfig(MotorInitConfig_t *config)
{
    if (config == NULL)
        return;

    if (config->min_duty < 0.0f)
        config->min_duty = MOTOR_DEFAULT_MIN_DUTY;

    if ((config->max_duty <= 0.0f) || (config->max_duty > 1.0f))
        config->max_duty = MOTOR_DEFAULT_MAX_DUTY;

    if (config->min_duty > config->max_duty)
    {
        config->min_duty = MOTOR_DEFAULT_MIN_DUTY;
        config->max_duty = MOTOR_DEFAULT_MAX_DUTY;
    }

    if (config->type == MOTOR_TYPE_SERVO)
    {
        if (config->servo_max_angle <= config->servo_min_angle)
        {
            config->servo_min_angle = MOTOR_DEFAULT_SERVO_MIN_ANGLE;
            config->servo_max_angle = MOTOR_DEFAULT_SERVO_MAX_ANGLE;
        }

        if (config->servo_max_pulse_us <= config->servo_min_pulse_us)
        {
            config->servo_min_pulse_us = MOTOR_DEFAULT_SERVO_MIN_PULSE_US;
            config->servo_max_pulse_us = MOTOR_DEFAULT_SERVO_MAX_PULSE_US;
        }
    }

    if (config->encoder != NULL)
        config->use_encoder = true;

    if (config->use_tb6612)
        config->use_reverse_pwm = false;

    if (config->type != MOTOR_TYPE_REDUCTION)
    {
        config->use_encoder = false;
        config->encoder     = NULL;
        config->use_tb6612  = false;
    }
}

/**
 * @brief   注册电机对应的PWM实例
 * @details 必定注册主PWM；当减速电机开启反向PWM时，额外注册反向PWM实例。
 * @param   motor 电机对象指针
 * @return  bool 成功返回true，失败返回false
 */
static bool MotorRegisterPwm(Motor_t *motor)
{
    PWM_Init_Config_s pwm_config;

    if (motor == NULL)
        return false;

    memset(&pwm_config, 0, sizeof(pwm_config));
    pwm_config.htim      = motor->init_config.pwm.htim;
    pwm_config.channel   = motor->init_config.pwm.channel;
    pwm_config.period    = motor->init_config.pwm.period;
    pwm_config.dutyratio = 0.0f;
    pwm_config.id        = motor;

    motor->pwm = PWMRegister(&pwm_config);
    if (motor->pwm == NULL)
        return false;

    if ((motor->init_config.type == MOTOR_TYPE_REDUCTION) &&
        (motor->init_config.use_reverse_pwm) &&
        (!motor->init_config.use_tb6612))
    {
        memset(&pwm_config, 0, sizeof(pwm_config));
        pwm_config.htim      = motor->init_config.reverse_pwm.htim;
        pwm_config.channel   = motor->init_config.reverse_pwm.channel;
        pwm_config.period    = motor->init_config.reverse_pwm.period;
        pwm_config.dutyratio = 0.0f;
        pwm_config.id        = motor;

        motor->reverse_pwm = PWMRegister(&pwm_config);
        if (motor->reverse_pwm == NULL)
            return false;
    }

    return true;
}

/**
 * @brief   同步编码器数据到电机对象缓存
 * @details 对外只同步电机实际转速，不透传编码器原始计数和方向。
 * @param   motor 电机对象指针
 */
static void MotorSyncEncoderData(Motor_t *motor)
{
    EncoderData_t encoder_data;

    if (motor == NULL)
        return;

    if (motor->encoder == NULL)
    {
        motor->data.speed_rps = 0.0f;
        return;
    }

    EncoderGetData(motor->encoder, &encoder_data);
    motor->data.speed_rps = encoder_data.speed_rps;
}

/**
 * @brief   应用减速电机输出
 * @details 根据是否启用反向PWM决定输入范围，完成占空比映射并更新方向/输出缓存。
 * @param   motor 电机对象指针
 * @param   speed 速度控制量
 */
static void MotorWriteGpio(const MotorGpioConfig_t *config, GPIO_PinState state)
{
    if (!MotorGpioConfigIsValid(config))
        return;

    HAL_GPIO_WritePin(config->GPIOx, config->GPIO_Pin, state);
}

static void MotorTb6612SetStandby(Motor_t *motor, bool enable)
{
    if ((motor == NULL) ||
        (!motor->init_config.use_tb6612) ||
        (!motor->init_config.tb6612.use_standby))
    {
        return;
    }

    MotorWriteGpio(&motor->init_config.tb6612.standby,
                   enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void MotorTb6612ApplyDirection(Motor_t *motor, MotorDirection_t direction)
{
    GPIO_PinState in1_state = GPIO_PIN_RESET;
    GPIO_PinState in2_state = GPIO_PIN_RESET;

    if ((motor == NULL) || (!motor->init_config.use_tb6612))
        return;

    if (direction == MOTOR_DIR_FORWARD)
    {
        in1_state = motor->init_config.tb6612.reversed ? GPIO_PIN_RESET : GPIO_PIN_SET;
        in2_state = motor->init_config.tb6612.reversed ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    else if (direction == MOTOR_DIR_REVERSE)
    {
        in1_state = motor->init_config.tb6612.reversed ? GPIO_PIN_SET : GPIO_PIN_RESET;
        in2_state = motor->init_config.tb6612.reversed ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    else if (motor->init_config.tb6612.brake_on_stop)
    {
        in1_state = GPIO_PIN_SET;
        in2_state = GPIO_PIN_SET;
    }

    MotorWriteGpio(&motor->init_config.tb6612.in1, in1_state);
    MotorWriteGpio(&motor->init_config.tb6612.in2, in2_state);
}

static void MotorApplyReductionOutput(Motor_t *motor, float speed)
{
    float magnitude;
    float duty;

    if ((motor->init_config.use_reverse_pwm) || (motor->init_config.use_tb6612))
        speed = float_constrain(speed, -1.0f, 1.0f);
    else
        speed = float_constrain(speed, 0.0f, 1.0f);

    magnitude = fabsf(speed);
    if (magnitude <= 0.0f)
        duty = 0.0f;
    else
        duty = motor->init_config.min_duty +
               ((motor->init_config.max_duty - motor->init_config.min_duty) * magnitude);

    if (speed > 0.0f)
    {
        if (motor->init_config.use_tb6612)
        {
            MotorTb6612SetStandby(motor, true);
            MotorTb6612ApplyDirection(motor, MOTOR_DIR_FORWARD);
        }
        PWMSetDutyRatio(motor->pwm, duty);
        if (motor->reverse_pwm != NULL)
            PWMSetDutyRatio(motor->reverse_pwm, 0.0f);
        motor->data.direction = MOTOR_DIR_FORWARD;
    }
    else if ((speed < 0.0f) &&
             ((motor->reverse_pwm != NULL) || (motor->init_config.use_tb6612)))
    {
        if (motor->init_config.use_tb6612)
        {
            MotorTb6612SetStandby(motor, true);
            MotorTb6612ApplyDirection(motor, MOTOR_DIR_REVERSE);
            PWMSetDutyRatio(motor->pwm, duty);
        }
        else
        {
            PWMSetDutyRatio(motor->pwm, 0.0f);
            PWMSetDutyRatio(motor->reverse_pwm, duty);
        }
        motor->data.direction = MOTOR_DIR_REVERSE;
    }
    else
    {
        PWMSetDutyRatio(motor->pwm, 0.0f);
        if (motor->reverse_pwm != NULL)
            PWMSetDutyRatio(motor->reverse_pwm, 0.0f);
        if (motor->init_config.use_tb6612)
            MotorTb6612ApplyDirection(motor, MOTOR_DIR_STOP);
        motor->data.direction = MOTOR_DIR_STOP;
    }

    motor->data.output = speed;
    motor->data.angle  = 0.0f;
}

/**
 * @brief   应用舵机脉宽输出
 * @details 将微秒脉宽转换为秒，再调用PWM脉宽设置接口下发。
 * @param   motor 电机对象指针
 * @param   pulse_width_us 脉宽（us）
 */
static void MotorApplyServoPulse(Motor_t *motor, float pulse_width_us)
{
    float pulse_width_s;

    pulse_width_s = pulse_width_us / MOTOR_US_PER_SECOND;
    PWMSetPulseWidth(motor->pwm, pulse_width_s);

    motor->data.direction = MOTOR_DIR_STOP;
}
