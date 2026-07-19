/**
 * @file    gimbal.c
 * @brief   二维云台组件实现
 * @details 负责双轴机械限位、运动换算、同步控制和反馈状态汇总。
 */
#include "gimbal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define GIMBAL_DEG_PER_MOTOR_RPM 6.0f
#define GIMBAL_INT32_MAX_FLOAT 2147483520.0f
#define GIMBAL_INT32_MIN_FLOAT (-2147483648.0f)

static void GimbalBindMethods(Gimbal_t *gimbal);
static bool GimbalConfigIsValid(const GimbalInitConfig_t *config);
static bool GimbalAxisConfigIsValid(const GimbalAxisConfig_t *axis);
static float GimbalClamp(float value, float minimum, float maximum);
static float GimbalAxisDirection(const GimbalAxisConfig_t *axis);
static bool GimbalAxisAngleToPulse(const GimbalAxisConfig_t *axis,
                                   float angle_deg,
                                   int32_t *pulse);
static float GimbalAxisMotorSpeed(const GimbalAxisConfig_t *axis,
                                  float axis_speed_dps);
static float GimbalAxisAngle(const GimbalAxisConfig_t *axis,
                             const Zdt42Data_t *motor_data);
static float GimbalAxisSpeed(const GimbalAxisConfig_t *axis,
                             const Zdt42Data_t *motor_data);
static bool GimbalTriggerSync(Gimbal_t *gimbal);
static void GimbalSyncData(Gimbal_t *gimbal);

/**
 * @brief   初始化二维云台对象
 * @param   gimbal 云台对象指针
 * @param   config 初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool GimbalInit(Gimbal_t *gimbal, const GimbalInitConfig_t *config)
{
    bool feedback_ok = true;

    if ((gimbal == NULL) || (config == NULL))
        return false;

    GimbalBindMethods(gimbal);
    if (gimbal->initialized)
        return true;
    if (!GimbalConfigIsValid(config))
        return false;

    gimbal->init_config = *config;
    gimbal->yaw_motor = config->yaw.motor;
    gimbal->pitch_motor = config->pitch.motor;
    memset(&gimbal->data, 0, sizeof(gimbal->data));
    gimbal->data.control_mode = GIMBAL_CONTROL_IDLE;
    gimbal->last_feedback_request_tick = HAL_GetTick();
    gimbal->initialized = true;

    if (config->feedback_period_ms > 0U)
    {
        feedback_ok = Zdt42SetAutoReturn(gimbal->yaw_motor,
                                          ZDT42_PARAM_POSITION,
                                          0U) && feedback_ok;
        feedback_ok = Zdt42SetAutoReturn(gimbal->yaw_motor,
                                          ZDT42_PARAM_SPEED,
                                          config->feedback_period_ms) && feedback_ok;
        feedback_ok = Zdt42SetAutoReturn(gimbal->pitch_motor,
                                          ZDT42_PARAM_POSITION,
                                          0U) && feedback_ok;
        feedback_ok = Zdt42SetAutoReturn(gimbal->pitch_motor,
                                          ZDT42_PARAM_SPEED,
                                          config->feedback_period_ms) && feedback_ok;
    }

    if (!feedback_ok)
    {
        gimbal->initialized = false;
        return false;
    }

    if (config->auto_enable && !GimbalEnable(gimbal, true))
    {
        gimbal->initialized = false;
        return false;
    }

    GimbalSyncData(gimbal);
    return true;
}

/** @brief 设置两个云台轴的使能状态 */
bool GimbalEnable(Gimbal_t *gimbal, bool enabled)
{
    bool yaw_ok;
    bool pitch_ok;

    if ((gimbal == NULL) || (!gimbal->initialized))
        return false;

    yaw_ok = Zdt42Enable(gimbal->yaw_motor, enabled, false);
    pitch_ok = Zdt42Enable(gimbal->pitch_motor, enabled, false);
    if (!(yaw_ok && pitch_ok))
    {
        if (enabled)
        {
            (void)Zdt42Enable(gimbal->yaw_motor, false, false);
            (void)Zdt42Enable(gimbal->pitch_motor, false, false);
        }
        gimbal->data.enabled = false;
        return false;
    }

    gimbal->data.enabled = enabled;
    if (!enabled)
    {
        gimbal->data.control_mode = GIMBAL_CONTROL_IDLE;
        gimbal->data.target_yaw_speed_dps = 0.0f;
        gimbal->data.target_pitch_speed_dps = 0.0f;
    }
    return true;
}

/**
 * @brief   设置二维云台目标角度
 * @details 目标角度先经过各轴机械限位，再以绝对位置命令同步下发。
 */
bool GimbalSetAngle(Gimbal_t *gimbal, float yaw_deg, float pitch_deg)
{
    float command_yaw;
    float command_pitch;
    int32_t yaw_pulse;
    int32_t pitch_pulse;

    if ((gimbal == NULL) || (!gimbal->initialized) || (!gimbal->data.enabled) ||
        (yaw_deg != yaw_deg) || (pitch_deg != pitch_deg))
        return false;

    command_yaw = GimbalClamp(yaw_deg,
                              gimbal->init_config.yaw.min_angle_deg,
                              gimbal->init_config.yaw.max_angle_deg);
    command_pitch = GimbalClamp(pitch_deg,
                                gimbal->init_config.pitch.min_angle_deg,
                                gimbal->init_config.pitch.max_angle_deg);
    if (!GimbalAxisAngleToPulse(&gimbal->init_config.yaw, command_yaw, &yaw_pulse) ||
        !GimbalAxisAngleToPulse(&gimbal->init_config.pitch, command_pitch, &pitch_pulse))
        return false;

    if (!Zdt42MovePosition(gimbal->yaw_motor,
                           yaw_pulse,
                           gimbal->init_config.position_speed_rpm,
                           gimbal->init_config.acceleration,
                           ZDT42_POSITION_ABSOLUTE,
                           true))
        return false;
    if (!Zdt42MovePosition(gimbal->pitch_motor,
                           pitch_pulse,
                           gimbal->init_config.position_speed_rpm,
                           gimbal->init_config.acceleration,
                           ZDT42_POSITION_ABSOLUTE,
                           true))
        return false;
    if (!GimbalTriggerSync(gimbal))
        return false;

    gimbal->data.target_yaw_deg = command_yaw;
    gimbal->data.target_pitch_deg = command_pitch;
    gimbal->data.target_yaw_speed_dps = 0.0f;
    gimbal->data.target_pitch_speed_dps = 0.0f;
    gimbal->data.target_limited = (command_yaw != yaw_deg) ||
                                  (command_pitch != pitch_deg);
    gimbal->data.motion_complete = false;
    gimbal->data.control_mode = GIMBAL_CONTROL_POSITION;
    return true;
}

/** @brief 设置水平轴目标角度并保持当前俯仰目标 */
bool GimbalSetYawAngle(Gimbal_t *gimbal, float yaw_deg)
{
    if ((gimbal == NULL) || (!gimbal->initialized))
        return false;
    return GimbalSetAngle(gimbal, yaw_deg, gimbal->data.target_pitch_deg);
}

/** @brief 设置俯仰轴目标角度并保持当前水平目标 */
bool GimbalSetPitchAngle(Gimbal_t *gimbal, float pitch_deg)
{
    if ((gimbal == NULL) || (!gimbal->initialized))
        return false;
    return GimbalSetAngle(gimbal, gimbal->data.target_yaw_deg, pitch_deg);
}

/** @brief 设置二维云台目标角速度 */
bool GimbalSetAngularVelocity(Gimbal_t *gimbal,
                              float yaw_speed_dps,
                              float pitch_speed_dps)
{
    float command_yaw_speed = yaw_speed_dps;
    float command_pitch_speed = pitch_speed_dps;
    float yaw_motor_rpm;
    float pitch_motor_rpm;

    if ((gimbal == NULL) || (!gimbal->initialized) || (!gimbal->data.enabled) ||
        (yaw_speed_dps != yaw_speed_dps) || (pitch_speed_dps != pitch_speed_dps))
        return false;

    GimbalSyncData(gimbal);
    if ((gimbal->init_config.feedback_timeout_ms > 0U) &&
        (!gimbal->data.communication_ok))
        return false;
    if (((gimbal->data.yaw_deg <= gimbal->init_config.yaw.min_angle_deg) &&
         (command_yaw_speed < 0.0f)) ||
        ((gimbal->data.yaw_deg >= gimbal->init_config.yaw.max_angle_deg) &&
         (command_yaw_speed > 0.0f)))
        command_yaw_speed = 0.0f;
    if (((gimbal->data.pitch_deg <= gimbal->init_config.pitch.min_angle_deg) &&
         (command_pitch_speed < 0.0f)) ||
        ((gimbal->data.pitch_deg >= gimbal->init_config.pitch.max_angle_deg) &&
         (command_pitch_speed > 0.0f)))
        command_pitch_speed = 0.0f;

    yaw_motor_rpm = GimbalAxisMotorSpeed(&gimbal->init_config.yaw, command_yaw_speed);
    pitch_motor_rpm = GimbalAxisMotorSpeed(&gimbal->init_config.pitch, command_pitch_speed);
    if ((yaw_motor_rpm > ZDT42_MAX_SPEED_RPM) || (yaw_motor_rpm < -ZDT42_MAX_SPEED_RPM) ||
        (pitch_motor_rpm > ZDT42_MAX_SPEED_RPM) || (pitch_motor_rpm < -ZDT42_MAX_SPEED_RPM))
        return false;

    if (!Zdt42SetSpeed(gimbal->yaw_motor,
                       yaw_motor_rpm,
                       gimbal->init_config.acceleration,
                       true))
        return false;
    if (!Zdt42SetSpeed(gimbal->pitch_motor,
                       pitch_motor_rpm,
                       gimbal->init_config.acceleration,
                       true))
        return false;
    if (!GimbalTriggerSync(gimbal))
        return false;

    gimbal->data.target_yaw_speed_dps = command_yaw_speed;
    gimbal->data.target_pitch_speed_dps = command_pitch_speed;
    gimbal->data.target_limited = (command_yaw_speed != yaw_speed_dps) ||
                                  (command_pitch_speed != pitch_speed_dps);
    gimbal->data.motion_complete = false;
    gimbal->data.control_mode = GIMBAL_CONTROL_VELOCITY;
    return true;
}

/** @brief 立即停止两个云台轴，电机保持使能 */
bool GimbalStop(Gimbal_t *gimbal)
{
    bool yaw_ok;
    bool pitch_ok;

    if ((gimbal == NULL) || (!gimbal->initialized))
        return false;

    yaw_ok = Zdt42Stop(gimbal->yaw_motor, false);
    pitch_ok = Zdt42Stop(gimbal->pitch_motor, false);
    if (!(yaw_ok && pitch_ok))
        return false;

    gimbal->data.target_yaw_speed_dps = 0.0f;
    gimbal->data.target_pitch_speed_dps = 0.0f;
    gimbal->data.control_mode = GIMBAL_CONTROL_IDLE;
    return true;
}

/** @brief 按各轴配置同步触发回零 */
bool GimbalHome(Gimbal_t *gimbal)
{
    bool yaw_ok;
    bool pitch_ok;

    if ((gimbal == NULL) || (!gimbal->initialized) || (!gimbal->data.enabled))
        return false;

    yaw_ok = Zdt42Home(gimbal->yaw_motor, gimbal->init_config.yaw.home_mode, true);
    pitch_ok = Zdt42Home(gimbal->pitch_motor, gimbal->init_config.pitch.home_mode, true);
    if (!(yaw_ok && pitch_ok && GimbalTriggerSync(gimbal)))
        return false;

    gimbal->data.motion_complete = false;
    gimbal->data.control_mode = GIMBAL_CONTROL_HOMING;
    return true;
}

/** @brief 将两个轴当前位置清零，并将云台坐标零偏重置为0 */
bool GimbalZero(Gimbal_t *gimbal)
{
    bool yaw_ok;
    bool pitch_ok;

    if ((gimbal == NULL) || (!gimbal->initialized))
        return false;

    yaw_ok = Zdt42ResetPosition(gimbal->yaw_motor);
    pitch_ok = Zdt42ResetPosition(gimbal->pitch_motor);
    if (!(yaw_ok && pitch_ok))
        return false;

    gimbal->init_config.yaw.position_offset_deg = 0.0f;
    gimbal->init_config.pitch.position_offset_deg = 0.0f;
    gimbal->data.target_yaw_deg = 0.0f;
    gimbal->data.target_pitch_deg = 0.0f;
    GimbalSyncData(gimbal);
    return true;
}

/** @brief 更新并汇总两个电机的云台级反馈状态 */
void GimbalUpdate(Gimbal_t *gimbal)
{
    uint32_t now_tick;

    if ((gimbal == NULL) || (!gimbal->initialized))
        return;

    now_tick = HAL_GetTick();
    if ((gimbal->init_config.feedback_period_ms > 0U) &&
        ((now_tick - gimbal->last_feedback_request_tick) >=
         gimbal->init_config.feedback_period_ms))
    {
        gimbal->last_feedback_request_tick = now_tick;
        (void)Zdt42ReadParameter(gimbal->yaw_motor, ZDT42_PARAM_POSITION);
        (void)Zdt42ReadParameter(gimbal->pitch_motor, ZDT42_PARAM_POSITION);
    }

    GimbalSyncData(gimbal);
    if ((gimbal->data.enabled) &&
        (gimbal->data.control_mode == GIMBAL_CONTROL_VELOCITY) &&
        (gimbal->init_config.feedback_timeout_ms > 0U) &&
        (!gimbal->data.communication_ok))
    {
        (void)GimbalStop(gimbal);
        gimbal->data.update_count++;
        return;
    }
    if ((gimbal->data.enabled) &&
        (gimbal->data.control_mode == GIMBAL_CONTROL_VELOCITY) &&
        (((gimbal->data.yaw_deg <= gimbal->init_config.yaw.min_angle_deg) &&
          (gimbal->data.target_yaw_speed_dps < 0.0f)) ||
         ((gimbal->data.yaw_deg >= gimbal->init_config.yaw.max_angle_deg) &&
          (gimbal->data.target_yaw_speed_dps > 0.0f)) ||
         ((gimbal->data.pitch_deg <= gimbal->init_config.pitch.min_angle_deg) &&
          (gimbal->data.target_pitch_speed_dps < 0.0f)) ||
         ((gimbal->data.pitch_deg >= gimbal->init_config.pitch.max_angle_deg) &&
          (gimbal->data.target_pitch_speed_dps > 0.0f))))
    {
        gimbal->data.target_limited = true;
        (void)GimbalStop(gimbal);
    }
    gimbal->data.update_count++;
}

/** @brief 获取二维云台运行数据的一致快照 */
void GimbalGetData(Gimbal_t *gimbal, GimbalData_t *data)
{
    if ((gimbal == NULL) || (data == NULL))
        return;

    taskENTER_CRITICAL();
    memcpy(data, &gimbal->data, sizeof(*data));
    taskEXIT_CRITICAL();
}

static bool GimbalConfigIsValid(const GimbalInitConfig_t *config)
{
    if (!GimbalAxisConfigIsValid(&config->yaw) ||
        !GimbalAxisConfigIsValid(&config->pitch) ||
        (config->yaw.motor == config->pitch.motor) ||
        (config->position_speed_rpm == 0U) ||
        (config->position_speed_rpm > (uint16_t)ZDT42_MAX_SPEED_RPM))
        return false;

    return true;
}

static bool GimbalAxisConfigIsValid(const GimbalAxisConfig_t *axis)
{
    int32_t pulse;

    if ((axis == NULL) || (axis->motor == NULL) || (!axis->motor->initialized) ||
        (axis->pulses_per_motor_rev != axis->pulses_per_motor_rev) ||
        (axis->motor_to_axis_ratio != axis->motor_to_axis_ratio) ||
        (axis->min_angle_deg != axis->min_angle_deg) ||
        (axis->max_angle_deg != axis->max_angle_deg) ||
        (axis->position_offset_deg != axis->position_offset_deg) ||
        (axis->pulses_per_motor_rev <= 0.0f) ||
        (axis->motor_to_axis_ratio <= 0.0f) ||
        (axis->min_angle_deg > axis->max_angle_deg) ||
        (axis->home_mode > ZDT42_HOME_LIMIT_SWITCH))
        return false;

    return GimbalAxisAngleToPulse(axis, axis->min_angle_deg, &pulse) &&
           GimbalAxisAngleToPulse(axis, axis->max_angle_deg, &pulse);
}

static bool GimbalAxisAngleToPulse(const GimbalAxisConfig_t *axis,
                                   float angle_deg,
                                   int32_t *pulse)
{
    float pulse_value;

    if ((axis == NULL) || (pulse == NULL))
        return false;

    pulse_value = (angle_deg - axis->position_offset_deg) *
                  axis->motor_to_axis_ratio *
                  axis->pulses_per_motor_rev /
                  360.0f * GimbalAxisDirection(axis);
    if ((pulse_value > GIMBAL_INT32_MAX_FLOAT) ||
        (pulse_value < GIMBAL_INT32_MIN_FLOAT))
        return false;

    *pulse = (int32_t)(pulse_value >= 0.0f ? pulse_value + 0.5f : pulse_value - 0.5f);
    return true;
}

static float GimbalAxisMotorSpeed(const GimbalAxisConfig_t *axis,
                                  float axis_speed_dps)
{
    return axis_speed_dps * axis->motor_to_axis_ratio /
           GIMBAL_DEG_PER_MOTOR_RPM * GimbalAxisDirection(axis);
}

static float GimbalAxisAngle(const GimbalAxisConfig_t *axis,
                             const Zdt42Data_t *motor_data)
{
    return motor_data->position_deg * GimbalAxisDirection(axis) /
           axis->motor_to_axis_ratio + axis->position_offset_deg;
}

static float GimbalAxisSpeed(const GimbalAxisConfig_t *axis,
                             const Zdt42Data_t *motor_data)
{
    return motor_data->speed_rpm * GIMBAL_DEG_PER_MOTOR_RPM *
           GimbalAxisDirection(axis) / axis->motor_to_axis_ratio;
}

static bool GimbalTriggerSync(Gimbal_t *gimbal)
{
    return Zdt42TriggerSync(gimbal->yaw_motor);
}

static void GimbalSyncData(Gimbal_t *gimbal)
{
    Zdt42Data_t yaw_data;
    Zdt42Data_t pitch_data;
    uint32_t now_tick;
    bool yaw_alive;
    bool pitch_alive;

    Zdt42GetData(gimbal->yaw_motor, &yaw_data);
    Zdt42GetData(gimbal->pitch_motor, &pitch_data);
    gimbal->data.yaw_deg = GimbalAxisAngle(&gimbal->init_config.yaw, &yaw_data);
    gimbal->data.pitch_deg = GimbalAxisAngle(&gimbal->init_config.pitch, &pitch_data);
    gimbal->data.yaw_speed_dps = GimbalAxisSpeed(&gimbal->init_config.yaw, &yaw_data);
    gimbal->data.pitch_speed_dps = GimbalAxisSpeed(&gimbal->init_config.pitch, &pitch_data);
    gimbal->data.motion_complete = yaw_data.motion_complete && pitch_data.motion_complete;

    if (gimbal->init_config.feedback_timeout_ms == 0U)
    {
        gimbal->data.communication_ok = true;
        return;
    }

    now_tick = HAL_GetTick();
    yaw_alive = (yaw_data.rx_frame_count > 0U) &&
                ((now_tick - yaw_data.last_rx_tick) <= gimbal->init_config.feedback_timeout_ms);
    pitch_alive = (pitch_data.rx_frame_count > 0U) &&
                  ((now_tick - pitch_data.last_rx_tick) <= gimbal->init_config.feedback_timeout_ms);
    gimbal->data.communication_ok = yaw_alive && pitch_alive;
}

static float GimbalClamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static float GimbalAxisDirection(const GimbalAxisConfig_t *axis)
{
    return axis->reversed ? -1.0f : 1.0f;
}

static void GimbalBindMethods(Gimbal_t *gimbal)
{
    gimbal->init = GimbalInit;
    gimbal->enable = GimbalEnable;
    gimbal->set_angle = GimbalSetAngle;
    gimbal->set_yaw_angle = GimbalSetYawAngle;
    gimbal->set_pitch_angle = GimbalSetPitchAngle;
    gimbal->set_angular_velocity = GimbalSetAngularVelocity;
    gimbal->stop = GimbalStop;
    gimbal->home = GimbalHome;
    gimbal->zero = GimbalZero;
    gimbal->update = GimbalUpdate;
    gimbal->get_data = GimbalGetData;
}
