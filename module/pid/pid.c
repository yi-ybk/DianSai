/**
 * @file    pid.c
 * @brief   PID控制组件实现
 * @details 实现通用位置式PID计算、限幅、参数更新和对象式方法绑定。
 */
#include "pid.h"
#include "user_lib_math.h"
#include <math.h>
#include <string.h>

static void PidBindMethods(Pid_t *pid);
static bool PidConfigIsValid(const PidInitConfig_t *config);
static float PidApplyOutputLimit(Pid_t *pid, float output);
static void PidApplyIntegralLimit(Pid_t *pid);

/**
 * @brief   初始化PID对象
 * @param   pid PID对象指针
 * @param   config PID初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool PidInit(Pid_t *pid, const PidInitConfig_t *config)
{
    if ((pid == NULL) || (config == NULL))
        return false;

    PidBindMethods(pid);
    if (pid->initialized)
        return true;

    pid->init_config = *config;
    if (!PidConfigIsValid(&pid->init_config))
        return false;

    memset(&pid->data, 0, sizeof(pid->data));
    pid->initialized = true;

    return true;
}

/**
 * @brief   计算PID输出
 * @details 输入dt_s小于等于0时不累加积分，也不计算微分项。
 * @param   pid PID对象指针
 * @param   feedback 反馈值
 * @param   target 目标值
 * @param   dt_s 计算周期(s)
 * @return  float PID输出
 */
float PidCalculate(Pid_t *pid, float feedback, float target, float dt_s)
{
    bool has_last_sample;
    float error;
    float derivative;
    float output;

    if ((pid == NULL) || (!pid->initialized))
        return 0.0f;

    if (dt_s < 0.0f)
        dt_s = 0.0f;

    has_last_sample = (pid->data.update_count > 0U);
    error = target - feedback;

    pid->data.target   = target;
    pid->data.feedback = feedback;
    pid->data.dt_s     = dt_s;
    pid->data.error    = error;
    pid->data.in_deadband = ((pid->init_config.deadband > 0.0f) &&
                             (fabsf(error) <= pid->init_config.deadband));

    if (pid->data.in_deadband)
    {
        error = 0.0f;
        pid->data.error = 0.0f;
        if (pid->init_config.reset_integral_on_deadband)
            pid->data.integral = 0.0f;
    }

    derivative = 0.0f;
    if ((dt_s > 0.0f) && has_last_sample)
    {
        if (pid->init_config.derivative_on_measurement)
            derivative = -(feedback - pid->data.last_feedback) / dt_s;
        else
            derivative = (error - pid->data.last_error) / dt_s;
    }

    if ((dt_s > 0.0f) && (!pid->data.in_deadband))
        pid->data.integral += error * dt_s;

    pid->data.derivative = derivative;
    pid->data.p_out = pid->init_config.kp * error;
    pid->data.i_out = pid->init_config.ki * pid->data.integral;
    PidApplyIntegralLimit(pid);
    pid->data.d_out = pid->init_config.kd * derivative;

    output = pid->data.p_out + pid->data.i_out + pid->data.d_out;
    output = PidApplyOutputLimit(pid, output);

    pid->data.last_error    = pid->data.error;
    pid->data.last_feedback = feedback;
    pid->data.last_output   = pid->data.output;
    pid->data.output        = output;
    pid->data.update_count++;

    return pid->data.output;
}

/**
 * @brief   复位PID运行数据
 * @param   pid PID对象指针
 */
void PidReset(Pid_t *pid)
{
    if (pid == NULL)
        return;

    memset(&pid->data, 0, sizeof(pid->data));
}

/**
 * @brief   设置PID目标值
 * @param   pid PID对象指针
 * @param   target 目标值
 */
void PidSetTarget(Pid_t *pid, float target)
{
    if ((pid == NULL) || (!pid->initialized))
        return;

    pid->data.target = target;
}

/**
 * @brief   设置PID参数
 * @param   pid PID对象指针
 * @param   kp 比例系数
 * @param   ki 积分系数
 * @param   kd 微分系数
 */
void PidSetParam(Pid_t *pid, float kp, float ki, float kd)
{
    if ((pid == NULL) || (!pid->initialized))
        return;

    pid->init_config.kp = kp;
    pid->init_config.ki = ki;
    pid->init_config.kd = kd;
}

/**
 * @brief   设置PID输出限幅
 * @param   pid PID对象指针
 * @param   enable 是否启用输出限幅
 * @param   output_min 输出最小值
 * @param   output_max 输出最大值
 */
void PidSetOutputLimit(Pid_t *pid, bool enable, float output_min, float output_max)
{
    if ((pid == NULL) || (!pid->initialized))
        return;

    if (enable && (output_min > output_max))
        return;

    pid->init_config.enable_output_limit = enable;
    pid->init_config.output_min = output_min;
    pid->init_config.output_max = output_max;
    pid->data.output = PidApplyOutputLimit(pid, pid->data.output);
}

/**
 * @brief   设置PID积分项限幅
 * @param   pid PID对象指针
 * @param   enable 是否启用积分项限幅
 * @param   integral_min 积分输出最小值
 * @param   integral_max 积分输出最大值
 */
void PidSetIntegralLimit(Pid_t *pid, bool enable, float integral_min, float integral_max)
{
    if ((pid == NULL) || (!pid->initialized))
        return;

    if (enable && (integral_min > integral_max))
        return;

    pid->init_config.enable_integral_limit = enable;
    pid->init_config.integral_min = integral_min;
    pid->init_config.integral_max = integral_max;
    PidApplyIntegralLimit(pid);
}

/**
 * @brief   获取PID运行数据
 * @param   pid PID对象指针
 * @param   data 数据输出目标
 */
void PidGetData(Pid_t *pid, PidData_t *data)
{
    if ((pid == NULL) || (data == NULL))
        return;

    memcpy(data, &pid->data, sizeof(*data));
}

/**
 * @brief   绑定对象方法
 * @param   pid PID对象指针
 */
static void PidBindMethods(Pid_t *pid)
{
    if (pid == NULL)
        return;

    pid->init               = PidInit;
    pid->calculate          = PidCalculate;
    pid->reset              = PidReset;
    pid->set_target         = PidSetTarget;
    pid->set_param          = PidSetParam;
    pid->set_output_limit   = PidSetOutputLimit;
    pid->set_integral_limit = PidSetIntegralLimit;
    pid->get_data           = PidGetData;
}

/**
 * @brief   校验PID初始化配置
 * @param   config PID初始化配置
 * @return  bool 合法返回true，否则返回false
 */
static bool PidConfigIsValid(const PidInitConfig_t *config)
{
    if (config == NULL)
        return false;

    if (config->deadband < 0.0f)
        return false;

    if (config->enable_output_limit && (config->output_min > config->output_max))
        return false;

    if (config->enable_integral_limit && (config->integral_min > config->integral_max))
        return false;

    return true;
}

/**
 * @brief   应用输出限幅
 * @param   pid PID对象指针
 * @param   output 输入输出值
 * @return  float 限幅后的输出值
 */
static float PidApplyOutputLimit(Pid_t *pid, float output)
{
    if ((pid == NULL) || (!pid->init_config.enable_output_limit))
        return output;

    return float_constrain(output,
                           pid->init_config.output_min,
                           pid->init_config.output_max);
}

/**
 * @brief   应用积分输出限幅
 * @param   pid PID对象指针
 */
static void PidApplyIntegralLimit(Pid_t *pid)
{
    if (pid == NULL)
        return;

    if (!pid->init_config.enable_integral_limit)
        return;

    pid->data.i_out = float_constrain(pid->data.i_out,
                                      pid->init_config.integral_min,
                                      pid->init_config.integral_max);

    if (pid->init_config.ki != 0.0f)
        pid->data.integral = pid->data.i_out / pid->init_config.ki;
    else
        pid->data.integral = 0.0f;
}
