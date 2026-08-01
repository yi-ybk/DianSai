/**
 * @file ball_control.c
 * @brief Low-overhead controller for a servo-driven ball-and-beam.
 */
#include "ball_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static float BallClamp(float value, float minimum, float maximum);
static float BallAbs(float value);
static bool BallFinite(float value);
static void BallUpdateSequence(BallControl_t *control, uint32_t now_tick);

bool BallControlInit(BallControl_t *control,
                     const BallControlInitConfig_t *config)
{
    if ((control == NULL) || (config == NULL) || (config->motor == NULL) ||
        (config->motor_to_rod_ratio <= 0.0f) ||
        (config->maximum_target_cm <= 0.0f) ||
        (config->maximum_rod_angle_deg <= 0.0f) ||
        (!config->motor->initialized) ||
        (config->motor->init_config.type != MOTOR_TYPE_SERVO) ||
        (config->servo_center_angle_deg <
         config->motor->init_config.servo_min_angle) ||
        (config->servo_center_angle_deg >
         config->motor->init_config.servo_max_angle) ||
        (config->maximum_servo_slew_deg_per_s <= 0.0f) ||
        (BallAbs(config->motor_direction) < 0.5f) ||
        (config->measurement_velocity_filter_s < 0.0f) ||
        (config->prediction_horizon_s < 0.0f) ||
        (config->sequence_tolerance_cm <= 0.0f) ||
        (config->control_period_ms == 0U) ||
        (config->motor_command_period_ms < config->control_period_ms) ||
        (config->measurement_timeout_ms == 0U) ||
        (config->sequence_hold_ms == 0U))
    {
        return false;
    }

    memset(control, 0, sizeof(*control));
    control->init = BallControlInit;
    control->start = BallControlStart;
    control->stop = BallControlStop;
    control->set_target = BallControlSetTarget;
    control->set_measurement = BallControlSetMeasurement;
    control->update = BallControlUpdate;
    control->get_data = BallControlGetData;
    control->init_config = *config;
    control->position_pid = (Pid_t){ PID_OBJECT_DEFAULT };
    if (!control->position_pid.init(&control->position_pid,
                                    &config->position_pid))
    {
        return false;
    }

    control->data.state = BALL_CONTROL_IDLE;
    control->data.servo_angle_command_deg = config->servo_center_angle_deg;
    control->previous_servo_angle_deg = config->servo_center_angle_deg;
    control->initialized = true;
    return true;
}

bool BallControlStart(BallControl_t *control,
                      BallControlProfile_t profile,
                      float hold_target_cm,
                      float positive_target_cm,
                      float negative_target_cm,
                      uint32_t now_tick)
{
    if ((control == NULL) || (!control->initialized) ||
        (profile > BALL_PROFILE_REQUIREMENT3) ||
        !BallFinite(hold_target_cm) ||
        !BallFinite(positive_target_cm) ||
        !BallFinite(negative_target_cm))
    {
        return false;
    }

    control->hold_target_cm = BallClamp(
        hold_target_cm,
        -control->init_config.maximum_target_cm,
        control->init_config.maximum_target_cm);
    control->positive_target_cm = BallClamp(
        positive_target_cm,
        -control->init_config.maximum_target_cm,
        control->init_config.maximum_target_cm);
    control->negative_target_cm = BallClamp(
        negative_target_cm,
        -control->init_config.maximum_target_cm,
        control->init_config.maximum_target_cm);

    control->position_pid.reset(&control->position_pid);
    control->previous_servo_angle_deg =
        control->init_config.servo_center_angle_deg;
    control->last_update_tick = now_tick;
    control->last_command_tick = now_tick;
    control->sequence_inside_tick = 0U;
    control->sequence_inside = false;

    taskENTER_CRITICAL();
    control->data.profile = profile;
    control->data.sequence =
        profile == BALL_PROFILE_REQUIREMENT3 ?
        BALL_SEQUENCE_TO_POSITIVE : BALL_SEQUENCE_HOLD;
    control->data.target_cm =
        profile == BALL_PROFILE_REQUIREMENT3 ?
        control->positive_target_cm : control->hold_target_cm;
    control->data.start_tick = now_tick;
    control->data.completion_time_ms = 0U;
    control->data.sequence_complete = false;
    control->data.measurement_stale = true;
    control->data.rod_angle_deg = 0.0f;
    control->data.servo_angle_command_deg =
        control->init_config.servo_center_angle_deg;
    control->data.state = BALL_CONTROL_ACTIVE;
    taskEXIT_CRITICAL();

    control->init_config.motor->start(control->init_config.motor);
    control->init_config.motor->set_angle(
        control->init_config.motor,
        control->init_config.servo_center_angle_deg);
    return true;
}

void BallControlStop(BallControl_t *control)
{
    if ((control == NULL) || (!control->initialized))
        return;

    control->init_config.motor->set_angle(
        control->init_config.motor,
        control->init_config.servo_center_angle_deg);

    control->position_pid.reset(&control->position_pid);
    control->previous_servo_angle_deg =
        control->init_config.servo_center_angle_deg;
    taskENTER_CRITICAL();
    control->data.state = BALL_CONTROL_IDLE;
    control->data.servo_angle_command_deg =
        control->init_config.servo_center_angle_deg;
    control->data.desired_rod_angle_deg = 0.0f;
    control->data.rod_angle_deg = 0.0f;
    taskEXIT_CRITICAL();
}

void BallControlSetTarget(BallControl_t *control, float target_cm)
{
    if ((control == NULL) || (!control->initialized) ||
        !BallFinite(target_cm))
    {
        return;
    }

    target_cm = BallClamp(target_cm,
                          -control->init_config.maximum_target_cm,
                          control->init_config.maximum_target_cm);
    taskENTER_CRITICAL();
    control->hold_target_cm = target_cm;
    if (control->data.profile == BALL_PROFILE_HOLD)
        control->data.target_cm = target_cm;
    taskEXIT_CRITICAL();
}

void BallControlSetMeasurement(BallControl_t *control,
                               float position_cm,
                               uint32_t measurement_tick)
{
    float velocity_cm_s = 0.0f;

    if ((control == NULL) || (!control->initialized) ||
        !BallFinite(position_cm))
    {
        return;
    }

    position_cm = BallClamp(position_cm,
                            -control->init_config.maximum_target_cm,
                            control->init_config.maximum_target_cm);

    if (control->data.measurement_valid &&
        (measurement_tick > control->previous_measurement_tick))
    {
        float dt_s =
            (float)(measurement_tick - control->previous_measurement_tick) *
            0.001f;
        float raw_velocity =
            (position_cm - control->previous_measurement_cm) / dt_s;
        float alpha = dt_s /
            (control->init_config.measurement_velocity_filter_s + dt_s);

        if (alpha > 1.0f)
            alpha = 1.0f;
        velocity_cm_s =
            control->data.measured_velocity_cm_s +
            alpha * (raw_velocity - control->data.measured_velocity_cm_s);
    }

    taskENTER_CRITICAL();
    control->previous_measurement_cm = position_cm;
    control->previous_measurement_tick = measurement_tick;
    control->data.measured_cm = position_cm;
    control->data.measured_velocity_cm_s = velocity_cm_s;
    control->data.measurement_tick = measurement_tick;
    control->data.measurement_valid = true;
    taskEXIT_CRITICAL();
}

void BallControlUpdate(BallControl_t *control, uint32_t now_tick)
{
    BallControlData_t snapshot;
    float dt_s;
    float predicted_cm;
    float desired_angle_deg;
    float servo_angle_deg;
    float maximum_step;
    bool measurement_stale;

    if ((control == NULL) || (!control->initialized) ||
        (control->data.state == BALL_CONTROL_IDLE) ||
        (control->data.state == BALL_CONTROL_FAULT))
    {
        return;
    }

    if ((now_tick - control->last_update_tick) <
        control->init_config.control_period_ms)
    {
        return;
    }
    dt_s = (float)(now_tick - control->last_update_tick) * 0.001f;
    if (dt_s > 0.05f)
        dt_s = 0.05f;
    control->last_update_tick = now_tick;

    BallUpdateSequence(control, now_tick);
    taskENTER_CRITICAL();
    snapshot = control->data;
    taskEXIT_CRITICAL();

    measurement_stale =
        (!snapshot.measurement_valid) ||
        ((now_tick - snapshot.measurement_tick) >
         control->init_config.measurement_timeout_ms);
    predicted_cm = snapshot.measured_cm;
    if (!measurement_stale)
    {
        predicted_cm += snapshot.measured_velocity_cm_s *
                        control->init_config.prediction_horizon_s;
        desired_angle_deg = control->position_pid.calculate(
            &control->position_pid,
            predicted_cm,
            snapshot.target_cm,
            dt_s);
    }
    else
    {
        desired_angle_deg = 0.0f;
        control->position_pid.reset(&control->position_pid);
    }

    desired_angle_deg = BallClamp(
        desired_angle_deg,
        -control->init_config.maximum_rod_angle_deg,
        control->init_config.maximum_rod_angle_deg);
    servo_angle_deg = control->init_config.servo_center_angle_deg +
        desired_angle_deg * control->init_config.motor_direction *
        control->init_config.motor_to_rod_ratio;
    servo_angle_deg = BallClamp(
        servo_angle_deg,
        control->init_config.motor->init_config.servo_min_angle,
        control->init_config.motor->init_config.servo_max_angle);

    maximum_step =
        control->init_config.maximum_servo_slew_deg_per_s * dt_s;
    servo_angle_deg = BallClamp(
        servo_angle_deg,
        control->previous_servo_angle_deg - maximum_step,
        control->previous_servo_angle_deg + maximum_step);

    taskENTER_CRITICAL();
    control->data.predicted_cm = predicted_cm;
    control->data.desired_rod_angle_deg = desired_angle_deg;
    control->data.rod_angle_deg =
        (servo_angle_deg - control->init_config.servo_center_angle_deg) *
        control->init_config.motor_direction /
        control->init_config.motor_to_rod_ratio;
    control->data.servo_angle_command_deg = servo_angle_deg;
    control->data.measurement_stale = measurement_stale;
    if (measurement_stale)
        control->data.stale_count++;
    control->data.update_count++;
    taskEXIT_CRITICAL();

    if ((now_tick - control->last_command_tick) >=
        control->init_config.motor_command_period_ms)
    {
        control->last_command_tick = now_tick;
        control->init_config.motor->set_angle(
            control->init_config.motor, servo_angle_deg);
        control->previous_servo_angle_deg = servo_angle_deg;
        control->data.command_count++;
    }
}

void BallControlGetData(BallControl_t *control, BallControlData_t *data)
{
    if ((control == NULL) || (data == NULL))
        return;

    taskENTER_CRITICAL();
    memcpy(data, &control->data, sizeof(*data));
    taskEXIT_CRITICAL();
}

static void BallUpdateSequence(BallControl_t *control, uint32_t now_tick)
{
    float error;

    if ((control->data.profile != BALL_PROFILE_REQUIREMENT3) ||
        (!control->data.measurement_valid) ||
        ((now_tick - control->data.measurement_tick) >
         control->init_config.measurement_timeout_ms))
    {
        return;
    }

    error = control->data.target_cm - control->data.measured_cm;
    if (BallAbs(error) <= control->init_config.sequence_tolerance_cm)
    {
        if (!control->sequence_inside)
        {
            control->sequence_inside = true;
            control->sequence_inside_tick = now_tick;
        }
        else if ((now_tick - control->sequence_inside_tick) >=
                 control->init_config.sequence_hold_ms)
        {
            control->sequence_inside = false;
            control->position_pid.reset(&control->position_pid);
            if (control->data.sequence == BALL_SEQUENCE_TO_POSITIVE)
            {
                control->data.sequence = BALL_SEQUENCE_TO_NEGATIVE;
                control->data.target_cm = control->negative_target_cm;
            }
            else if (control->data.sequence == BALL_SEQUENCE_TO_NEGATIVE)
            {
                control->data.sequence = BALL_SEQUENCE_COMPLETE;
                control->data.sequence_complete = true;
                control->data.completion_time_ms =
                    now_tick - control->data.start_tick;
            }
        }
    }
    else
    {
        control->sequence_inside = false;
    }
}

static float BallClamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static float BallAbs(float value)
{
    return value < 0.0f ? -value : value;
}

static bool BallFinite(float value)
{
    return (value == value) && (value <= 3.4028234e38f) &&
           (value >= -3.4028234e38f);
}
