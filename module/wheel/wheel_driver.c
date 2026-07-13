/**
 * @file    wheel_driver.c
 * @brief   驱动轮模块实现
 * @details 将减速电机、编码器反馈和可选速度PID组合为轮级控制对象，完成速度、角速度和里程换算。
 */
#include "wheel_driver.h"
#include <string.h>

#define WHEEL_DEFAULT_RATIO 1.0f              /**< 默认编码器到轮子传动比 */
#define WHEEL_DEFAULT_MAX_SPEED_MPS 1.0f      /**< 默认开环映射最大线速度 */
#define WHEEL_DEFAULT_OUTPUT_MIN (-1.0f)      /**< 默认输出下限 */
#define WHEEL_DEFAULT_OUTPUT_MAX 1.0f         /**< 默认输出上限 */
#define WHEEL_TWO_PI 6.2831853071795864769f   /**< 2*pi */

static void WheelBindMethods(Wheel_t *wheel);
static bool WheelConfigIsValid(const WheelInitConfig_t *config);
static void WheelNormalizeConfig(WheelInitConfig_t *config);
static float WheelClamp(float value, float min, float max);
static float WheelGetDirectionSign(const Wheel_t *wheel);
static float WheelLinearToAngular(const Wheel_t *wheel, float speed_mps);
static float WheelAngularToLinear(const Wheel_t *wheel, float speed_radps);
static float WheelLinearToOutput(const Wheel_t *wheel, float speed_mps);
static void WheelApplyMotorOutput(Wheel_t *wheel, float output);
static void WheelSyncData(Wheel_t *wheel);

/**
 * @brief   初始化驱动轮对象
 * @param   wheel 驱动轮对象指针
 * @param   config 驱动轮初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool WheelInit(Wheel_t *wheel, const WheelInitConfig_t *config)
{
    if ((wheel == NULL) || (config == NULL))
        return false;

    WheelBindMethods(wheel);
    if (wheel->initialized)
        return true;

    wheel->init_config = *config;
    WheelNormalizeConfig(&wheel->init_config);
    if (!WheelConfigIsValid(&wheel->init_config))
        return false;

    wheel->motor     = wheel->init_config.motor;
    wheel->speed_pid = wheel->init_config.use_speed_pid ? wheel->init_config.speed_pid : NULL;

    memset(&wheel->data, 0, sizeof(wheel->data));
    wheel->data.control_mode = WHEEL_CONTROL_OPEN_LOOP;

    if (wheel->speed_pid != NULL)
        PidReset(wheel->speed_pid);

    wheel->initialized = true;
    if (wheel->init_config.auto_start)
        WheelStart(wheel);
    else
        WheelSyncData(wheel);

    return true;
}

/**
 * @brief   启动驱动轮
 * @param   wheel 驱动轮对象指针
 */
void WheelStart(Wheel_t *wheel)
{
    if ((wheel == NULL) || (!wheel->initialized))
        return;

    MotorStart(wheel->motor);
    wheel->data.enabled = true;
    WheelSyncData(wheel);
}

/**
 * @brief   停止驱动轮
 * @param   wheel 驱动轮对象指针
 */
void WheelStop(Wheel_t *wheel)
{
    if ((wheel == NULL) || (!wheel->initialized))
        return;

    MotorStop(wheel->motor);
    if (wheel->speed_pid != NULL)
        PidReset(wheel->speed_pid);

    wheel->data.enabled = false;
    wheel->data.target_linear_speed_mps = 0.0f;
    wheel->data.target_angular_speed_radps = 0.0f;
    wheel->data.motor_output = 0.0f;
    wheel->data.pid_output = 0.0f;
    WheelSyncData(wheel);
}

/**
 * @brief   设置开环输出
 * @param   wheel 驱动轮对象指针
 * @param   output 轮子逻辑输出，通常范围为-1~1
 */
void WheelSetOutput(Wheel_t *wheel, float output)
{
    if ((wheel == NULL) || (!wheel->initialized))
        return;

    wheel->data.control_mode = WHEEL_CONTROL_OPEN_LOOP;
    wheel->data.target_linear_speed_mps = 0.0f;
    wheel->data.target_angular_speed_radps = 0.0f;
    wheel->data.pid_output = 0.0f;
    WheelApplyMotorOutput(wheel, output);
    WheelSyncData(wheel);
}

/**
 * @brief   设置目标线速度
 * @param   wheel 驱动轮对象指针
 * @param   speed_mps 目标线速度(m/s)
 */
void WheelSetLinearSpeed(Wheel_t *wheel, float speed_mps)
{
    if ((wheel == NULL) || (!wheel->initialized))
        return;

    wheel->data.target_linear_speed_mps = speed_mps;
    wheel->data.target_angular_speed_radps = WheelLinearToAngular(wheel, speed_mps);

    if (wheel->speed_pid != NULL)
    {
        wheel->data.control_mode = WHEEL_CONTROL_SPEED;
        PidSetTarget(wheel->speed_pid, speed_mps);
    }
    else
    {
        wheel->data.control_mode = WHEEL_CONTROL_OPEN_LOOP;
        WheelApplyMotorOutput(wheel, WheelLinearToOutput(wheel, speed_mps));
        WheelSyncData(wheel);
    }
}

/**
 * @brief   设置目标角速度
 * @param   wheel 驱动轮对象指针
 * @param   speed_radps 目标角速度(rad/s)
 */
void WheelSetAngularSpeed(Wheel_t *wheel, float speed_radps)
{
    if ((wheel == NULL) || (!wheel->initialized))
        return;

    WheelSetLinearSpeed(wheel, WheelAngularToLinear(wheel, speed_radps));
}

/**
 * @brief   更新驱动轮状态和闭环输出
 * @details 应按固定周期调用；闭环模式下会先刷新编码器速度，再由PID计算并下发电机输出。
 * @param   wheel 驱动轮对象指针
 * @param   dt_s 更新周期(s)
 */
void WheelUpdate(Wheel_t *wheel, float dt_s)
{
    float output;

    if ((wheel == NULL) || (!wheel->initialized))
        return;

    if (dt_s < 0.0f)
        dt_s = 0.0f;

    MotorUpdate(wheel->motor, dt_s);
    WheelSyncData(wheel);

    if ((wheel->data.enabled) &&
        (wheel->data.control_mode == WHEEL_CONTROL_SPEED) &&
        (wheel->speed_pid != NULL))
    {
        output = PidCalculate(wheel->speed_pid,
                              wheel->data.linear_speed_mps,
                              wheel->data.target_linear_speed_mps,
                              dt_s);
        wheel->data.pid_output = output;
        WheelApplyMotorOutput(wheel, output);
    }

    wheel->data.update_count++;
}

/**
 * @brief   复位驱动轮里程
 * @param   wheel 驱动轮对象指针
 */
void WheelResetOdometry(Wheel_t *wheel)
{
    if ((wheel == NULL) || (!wheel->initialized))
        return;

    MotorResetEncoder(wheel->motor);
    wheel->data.distance_m = 0.0f;
    wheel->data.angle_rad = 0.0f;
    wheel->data.encoder_count = 0;
    wheel->data.encoder_delta = 0;
    WheelSyncData(wheel);
}

/**
 * @brief   获取驱动轮运行数据
 * @param   wheel 驱动轮对象指针
 * @param   data 数据输出目标
 */
void WheelGetData(Wheel_t *wheel, WheelData_t *data)
{
    if ((wheel == NULL) || (data == NULL))
        return;

    if (wheel->initialized)
        WheelSyncData(wheel);

    memcpy(data, &wheel->data, sizeof(*data));
}

/**
 * @brief   绑定对象方法
 * @param   wheel 驱动轮对象指针
 */
static void WheelBindMethods(Wheel_t *wheel)
{
    if (wheel == NULL)
        return;

    wheel->init              = WheelInit;
    wheel->start             = WheelStart;
    wheel->stop              = WheelStop;
    wheel->set_output        = WheelSetOutput;
    wheel->set_linear_speed  = WheelSetLinearSpeed;
    wheel->set_angular_speed = WheelSetAngularSpeed;
    wheel->update            = WheelUpdate;
    wheel->reset_odometry    = WheelResetOdometry;
    wheel->get_data          = WheelGetData;
}

/**
 * @brief   校验驱动轮初始化配置
 * @param   config 驱动轮初始化配置
 * @return  bool 合法返回true，否则返回false
 */
static bool WheelConfigIsValid(const WheelInitConfig_t *config)
{
    if ((config == NULL) || (config->motor == NULL) || (!config->motor->initialized))
        return false;

    if (config->motor->init_config.type != MOTOR_TYPE_REDUCTION)
        return false;

    if ((config->radius_m <= 0.0f) ||
        (config->encoder_to_wheel_ratio <= 0.0f) ||
        (config->max_linear_speed_mps <= 0.0f))
    {
        return false;
    }

    if (config->output_min > config->output_max)
        return false;

    if (config->use_speed_pid &&
        ((config->speed_pid == NULL) || (!config->speed_pid->initialized)))
    {
        return false;
    }

    return true;
}

/**
 * @brief   归一化驱动轮配置
 * @param   config 驱动轮初始化配置
 */
static void WheelNormalizeConfig(WheelInitConfig_t *config)
{
    if (config == NULL)
        return;

    if (config->encoder_to_wheel_ratio <= 0.0f)
        config->encoder_to_wheel_ratio = WHEEL_DEFAULT_RATIO;

    if (config->max_linear_speed_mps <= 0.0f)
        config->max_linear_speed_mps = WHEEL_DEFAULT_MAX_SPEED_MPS;

    if ((config->output_min == 0.0f) && (config->output_max == 0.0f))
    {
        config->output_min = WHEEL_DEFAULT_OUTPUT_MIN;
        config->output_max = WHEEL_DEFAULT_OUTPUT_MAX;
    }

    if (config->output_min < WHEEL_DEFAULT_OUTPUT_MIN)
        config->output_min = WHEEL_DEFAULT_OUTPUT_MIN;

    if (config->output_max > WHEEL_DEFAULT_OUTPUT_MAX)
        config->output_max = WHEEL_DEFAULT_OUTPUT_MAX;

    if (config->output_min > config->output_max)
    {
        config->output_min = WHEEL_DEFAULT_OUTPUT_MIN;
        config->output_max = WHEEL_DEFAULT_OUTPUT_MAX;
    }
}

/**
 * @brief   浮点数限幅
 * @param   value 输入值
 * @param   min 最小值
 * @param   max 最大值
 * @return  float 限幅后的值
 */
static float WheelClamp(float value, float min, float max)
{
    if (value < min)
        return min;

    if (value > max)
        return max;

    return value;
}

/**
 * @brief   获取轮子逻辑方向符号
 * @param   wheel 驱动轮对象指针
 * @return  float 方向符号
 */
static float WheelGetDirectionSign(const Wheel_t *wheel)
{
    if ((wheel != NULL) && (wheel->init_config.reversed))
        return -1.0f;

    return 1.0f;
}

/**
 * @brief   将线速度转换为角速度
 * @param   wheel 驱动轮对象指针
 * @param   speed_mps 线速度(m/s)
 * @return  float 角速度(rad/s)
 */
static float WheelLinearToAngular(const Wheel_t *wheel, float speed_mps)
{
    if ((wheel == NULL) || (wheel->init_config.radius_m <= 0.0f))
        return 0.0f;

    return speed_mps / wheel->init_config.radius_m;
}

/**
 * @brief   将角速度转换为线速度
 * @param   wheel 驱动轮对象指针
 * @param   speed_radps 角速度(rad/s)
 * @return  float 线速度(m/s)
 */
static float WheelAngularToLinear(const Wheel_t *wheel, float speed_radps)
{
    if (wheel == NULL)
        return 0.0f;

    return speed_radps * wheel->init_config.radius_m;
}

/**
 * @brief   将线速度映射为开环输出
 * @param   wheel 驱动轮对象指针
 * @param   speed_mps 目标线速度(m/s)
 * @return  float 开环输出
 */
static float WheelLinearToOutput(const Wheel_t *wheel, float speed_mps)
{
    if ((wheel == NULL) || (wheel->init_config.max_linear_speed_mps <= 0.0f))
        return 0.0f;

    return speed_mps / wheel->init_config.max_linear_speed_mps;
}

/**
 * @brief   下发电机输出
 * @param   wheel 驱动轮对象指针
 * @param   output 轮子逻辑输出
 */
static void WheelApplyMotorOutput(Wheel_t *wheel, float output)
{
    float logical_output;
    float motor_output;

    if ((wheel == NULL) || (wheel->motor == NULL))
        return;

    logical_output = WheelClamp(output,
                                wheel->init_config.output_min,
                                wheel->init_config.output_max);
    motor_output = logical_output * WheelGetDirectionSign(wheel);
    MotorSetSpeed(wheel->motor, motor_output);
    wheel->data.motor_output = motor_output;
}

/**
 * @brief   同步轮子运行数据
 * @param   wheel 驱动轮对象指针
 */
static void WheelSyncData(Wheel_t *wheel)
{
    MotorData_t motor_data;
    float direction_sign;
    float wheel_revolutions;
    float encoder_cpr;

    if ((wheel == NULL) || (wheel->motor == NULL))
        return;

    MotorGetData(wheel->motor, &motor_data);
    direction_sign = WheelGetDirectionSign(wheel);

    wheel->data.enabled       = motor_data.enabled;
    wheel->data.encoder_count = motor_data.encoder_count;
    wheel->data.encoder_delta = motor_data.encoder_delta;
    wheel->data.motor_output  = motor_data.output;

    if (wheel->init_config.reversed)
    {
        if (motor_data.direction == MOTOR_DIR_FORWARD)
            wheel->data.direction = MOTOR_DIR_REVERSE;
        else if (motor_data.direction == MOTOR_DIR_REVERSE)
            wheel->data.direction = MOTOR_DIR_FORWARD;
        else
            wheel->data.direction = MOTOR_DIR_STOP;
    }
    else
    {
        wheel->data.direction = motor_data.direction;
    }

    wheel->data.wheel_speed_rps =
        (motor_data.encoder_speed_rps / wheel->init_config.encoder_to_wheel_ratio) * direction_sign;
    wheel->data.angular_speed_radps = wheel->data.wheel_speed_rps * WHEEL_TWO_PI;
    wheel->data.linear_speed_mps = wheel->data.angular_speed_radps * wheel->init_config.radius_m;

    if ((wheel->motor->encoder != NULL) &&
        (wheel->motor->encoder->init_config.counts_per_rev > 0.0f))
    {
        encoder_cpr = wheel->motor->encoder->init_config.counts_per_rev;
        wheel_revolutions = ((float)motor_data.encoder_count / encoder_cpr) /
                            wheel->init_config.encoder_to_wheel_ratio;
        wheel_revolutions *= direction_sign;
        wheel->data.angle_rad = wheel_revolutions * WHEEL_TWO_PI;
        wheel->data.distance_m = wheel->data.angle_rad * wheel->init_config.radius_m;
    }
}
