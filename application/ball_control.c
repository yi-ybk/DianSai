/**
 * @file ball_control.c
 * @brief Low-overhead cascaded controller for a single-axis ball-and-beam.
 */
#include "ball_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static float BallClamp(float value, float minimum, float maximum);
static float BallAbs(float value);
static bool BallFinite(float value);
static float BallMotorOffsetToRodAngle(const BallControl_t *control,
                                       float motor_offset_deg);
static bool BallSendSpeed(BallControl_t *control, float speed_rpm);
static void BallMotorOff(BallControl_t *control);
static void BallUpdateSequence(BallControl_t *control, uint32_t now_tick);

bool BallControlInit(BallControl_t *control,
                     const BallControlInitConfig_t *config)
{
    if ((control == NULL) || (config == NULL) || (config->motor == NULL) ||
        (config->motor_to_rod_positive_linear <= 0.0f) ||
        (config->motor_to_rod_negative_linear <= 0.0f) ||
        (!BallFinite(config->motor_to_rod_positive_quadratic)) ||
        (!BallFinite(config->motor_to_rod_negative_quadratic)) ||
        (config->minimum_motor_offset_deg >= 0.0f) ||
        (config->maximum_motor_offset_deg <= 0.0f) ||
        (config->maximum_target_cm <= 0.0f) ||
        (config->maximum_rod_angle_deg <= 0.0f) ||
        (config->maximum_motor_speed_rpm <= 0.0f) ||
        (config->motor_speed_slew_rpm_per_s <= 0.0f) ||
        (BallAbs(config->motor_direction) < 0.5f) ||
        (BallAbs(config->position_to_rod_direction) < 0.5f) ||
        (config->measurement_velocity_filter_s < 0.0f) ||
        (config->prediction_horizon_s < 0.0f) ||
        (config->sequence_tolerance_cm <= 0.0f) ||
        (config->minimum_drive_angle_deg <= 0.0f) ||
        (config->minimum_drive_angle_deg >
         config->maximum_rod_angle_deg) ||
        (config->minimum_drive_error_cm <= 0.0f) ||
        (config->minimum_drive_velocity_cm_s <= 0.0f) ||
        (config->stall_position_epsilon_cm <= 0.0f) ||
        (config->disturbance_angle_deg <= 0.0f) ||
        (config->disturbance_angle_deg >
         config->maximum_rod_angle_deg) ||
        (config->lost_search_angle_deg <= 0.0f) ||
        (config->lost_search_angle_deg >
         config->maximum_rod_angle_deg) ||
        (config->return_ball_tolerance_cm <= 0.0f) ||
        (config->return_tolerance_deg <= 0.0f) ||
        (config->control_period_ms == 0U) ||
        (config->motor_command_period_ms < config->control_period_ms) ||
        (config->motor_feedback_period_ms == 0U) ||
        (config->motor_feedback_timeout_ms <
         (2U * config->motor_feedback_period_ms)) ||
        (config->measurement_timeout_ms == 0U) ||
        (config->arming_timeout_ms == 0U) ||
        (config->sequence_hold_ms == 0U) ||
        (config->stall_detection_ms == 0U) ||
        (config->disturbance_duration_ms == 0U) ||
        (config->lost_search_step_ms == 0U) ||
        (config->lost_search_timeout_ms <
         config->lost_search_step_ms) ||
        (config->return_ball_hold_ms == 0U) ||
        (config->return_hold_ms == 0U) ||
        (config->return_timeout_ms < config->return_hold_ms))
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
    control->angle_pid = (Pid_t){ PID_OBJECT_DEFAULT };
    if (!control->position_pid.init(&control->position_pid,
                                    &config->position_pid) ||
        !control->angle_pid.init(&control->angle_pid,
                                 &config->angle_pid))
    {
        return false;
    }

    control->data.state = BALL_CONTROL_IDLE;
    if (!config->motor->clear_fault(config->motor) ||
        !config->motor->stop(config->motor, false) ||
        !config->motor->enable(config->motor, true, false) ||
        !config->motor->set_auto_return(
            config->motor, ZDT42_PARAM_POSITION, 100U))
    {
        return false;
    }
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
        (control->data.state != BALL_CONTROL_IDLE) ||
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
    control->angle_pid.reset(&control->angle_pid);
    control->previous_speed_command_rpm = 0.0f;
    control->desired_angle_command_deg = 0.0f;
    control->requested_speed_command_rpm = 0.0f;
    control->last_outer_measurement_tick = 0U;
    control->last_inner_feedback_tick = 0U;
    control->last_update_tick = now_tick;
    control->last_command_tick = now_tick;
    control->last_feedback_request_tick = now_tick;
    control->last_position_feedback_tick = now_tick;
    control->sequence_inside_tick = 0U;
    control->sequence_inside = false;
    control->stall_reference_cm = control->data.measured_cm;
    control->stall_reference_tick = now_tick;
    control->disturbance_start_tick = 0U;
    control->measurement_stale_start_tick = 0U;
    control->disturbance_active = false;
    control->return_start_tick = 0U;
    control->return_inside_tick = 0U;
    control->return_inside = false;
    control->return_leveling = false;
    control->return_forced_level = false;

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
    control->data.motor_speed_command_rpm = 0.0f;
    control->data.state = BALL_CONTROL_ARMING;
    taskEXIT_CRITICAL();

    if (!control->init_config.motor->read_parameter(
            control->init_config.motor, ZDT42_PARAM_POSITION))
    {
        control->data.command_error_count++;
        control->data.state = BALL_CONTROL_FAULT;
        return false;
    }
    return true;
}

void BallControlStop(BallControl_t *control)
{
    uint32_t now_tick;

    if ((control == NULL) || (!control->initialized))
        return;

    if ((control->data.state == BALL_CONTROL_ACTIVE) ||
        (control->data.state == BALL_CONTROL_RETURNING))
    {
        now_tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        control->position_pid.reset(&control->position_pid);
        control->angle_pid.reset(&control->angle_pid);
        control->previous_speed_command_rpm = 0.0f;
        control->desired_angle_command_deg = 0.0f;
        control->requested_speed_command_rpm = 0.0f;
        control->return_start_tick = now_tick;
        control->return_inside_tick = 0U;
        control->return_inside = false;
        control->return_leveling = false;
        control->return_forced_level = false;
        control->disturbance_active = false;
        taskENTER_CRITICAL();
        control->data.state = BALL_CONTROL_RETURNING;
        control->data.target_cm = control->hold_target_cm;
        control->data.motor_speed_command_rpm = 0.0f;
        control->data.desired_rod_angle_deg = 0.0f;
        taskEXIT_CRITICAL();
        return;
    }

    BallMotorOff(control);
    control->position_pid.reset(&control->position_pid);
    control->angle_pid.reset(&control->angle_pid);
    control->previous_speed_command_rpm = 0.0f;
    control->desired_angle_command_deg = 0.0f;
    control->requested_speed_command_rpm = 0.0f;
    taskENTER_CRITICAL();
    control->data.state = BALL_CONTROL_IDLE;
    control->data.motor_speed_command_rpm = 0.0f;
    control->data.desired_rod_angle_deg = 0.0f;
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
    Zdt42Data_t motor_data;
    BallControlData_t snapshot;
    float predicted_cm;
    float desired_angle_deg;
    float rod_angle_deg;
    float speed_rpm;
    float maximum_step;
    float motor_offset_deg;
    float angle_error_deg;
    float position_error_cm;
    float outer_dt_s;
    float inner_dt_s;
    bool measurement_stale;
    bool new_measurement;
    bool new_motor_feedback;
    bool returning;

    if ((control == NULL) || (!control->initialized))
    {
        return;
    }

    if (control->data.state == BALL_CONTROL_IDLE)
    {
        if ((now_tick - control->last_feedback_request_tick) >= 100U)
        {
            control->last_feedback_request_tick = now_tick;
            (void)control->init_config.motor->read_parameter(
                control->init_config.motor, ZDT42_PARAM_POSITION);
        }
        return;
    }
    if (control->data.state == BALL_CONTROL_FAULT)
        return;

    if ((now_tick - control->last_update_tick) <
        control->init_config.control_period_ms)
    {
        return;
    }
    control->last_update_tick = now_tick;

    control->init_config.motor->get_data(
        control->init_config.motor, &motor_data);
    if ((motor_data.position_rx_tick != 0U) &&
        ((uint32_t)(motor_data.position_rx_tick -
                    control->last_position_feedback_tick) <
         0x80000000UL))
    {
        control->last_position_feedback_tick =
            motor_data.position_rx_tick;
    }

    if (control->data.state == BALL_CONTROL_ARMING)
    {
        if ((motor_data.position_rx_tick != 0U) &&
            ((uint32_t)(motor_data.position_rx_tick -
                        control->data.start_tick) < 0x80000000UL))
        {
            control->data.motor_zero_deg = motor_data.position_deg;
            if (!control->init_config.motor->enable(
                    control->init_config.motor, true, false) ||
                !control->init_config.motor->set_auto_return(
                    control->init_config.motor,
                    ZDT42_PARAM_POSITION,
                    control->init_config.motor_feedback_period_ms))
            {
                (void)control->init_config.motor->stop(
                    control->init_config.motor, false);
                (void)control->init_config.motor->enable(
                    control->init_config.motor, false, false);
                control->data.command_error_count++;
                control->data.state = BALL_CONTROL_FAULT;
                return;
            }
            control->data.state = BALL_CONTROL_ACTIVE;
        }
        else if ((now_tick - control->data.start_tick) >=
                 control->init_config.arming_timeout_ms)
        {
            control->data.state = BALL_CONTROL_FAULT;
            return;
        }
        else if ((now_tick - control->last_feedback_request_tick) >= 100U)
        {
            control->last_feedback_request_tick = now_tick;
            if (!control->init_config.motor->read_parameter(
                    control->init_config.motor, ZDT42_PARAM_POSITION))
            {
                control->data.command_error_count++;
            }
        }
        return;
    }

    if ((now_tick - control->last_position_feedback_tick) >
        control->init_config.motor_feedback_timeout_ms)
    {
        BallMotorOff(control);
        control->data.command_error_count++;
        control->data.motor_speed_command_rpm = 0.0f;
        control->data.state = BALL_CONTROL_FAULT;
        return;
    }

    returning = control->data.state == BALL_CONTROL_RETURNING;
    if (!returning)
        BallUpdateSequence(control, now_tick);
    taskENTER_CRITICAL();
    snapshot = control->data;
    taskEXIT_CRITICAL();

    measurement_stale =
        (!snapshot.measurement_valid) ||
        ((now_tick - snapshot.measurement_tick) >
         control->init_config.measurement_timeout_ms);
    predicted_cm = snapshot.measured_cm;
    new_measurement =
        (!measurement_stale) &&
        (snapshot.measurement_tick !=
         control->last_outer_measurement_tick);
    if (returning && control->return_leveling)
    {
        desired_angle_deg = 0.0f;
        predicted_cm = snapshot.measured_cm;
        control->desired_angle_command_deg = 0.0f;
        control->position_pid.reset(&control->position_pid);
    }
    else if (new_measurement)
    {
        outer_dt_s =
            control->last_outer_measurement_tick == 0U ?
            (float)control->init_config.control_period_ms * 0.001f :
            (float)(snapshot.measurement_tick -
                    control->last_outer_measurement_tick) * 0.001f;
        if (outer_dt_s < 0.005f)
            outer_dt_s = 0.005f;
        if (outer_dt_s > 0.10f)
            outer_dt_s = 0.10f;
        predicted_cm += snapshot.measured_velocity_cm_s *
                        control->init_config.prediction_horizon_s;
        desired_angle_deg = control->position_pid.calculate(
            &control->position_pid,
            predicted_cm,
            snapshot.target_cm,
            outer_dt_s);
        desired_angle_deg *=
            control->init_config.position_to_rod_direction;
        desired_angle_deg = BallClamp(
            desired_angle_deg,
            -control->init_config.maximum_rod_angle_deg,
            control->init_config.maximum_rod_angle_deg);
        position_error_cm = snapshot.target_cm - predicted_cm;
        if ((BallAbs(position_error_cm) >=
             control->init_config.minimum_drive_error_cm) &&
            (BallAbs(snapshot.measured_velocity_cm_s) <=
             control->init_config.minimum_drive_velocity_cm_s) &&
            ((desired_angle_deg * position_error_cm *
              control->init_config.position_to_rod_direction) > 0.0f) &&
            (BallAbs(desired_angle_deg) <
             control->init_config.minimum_drive_angle_deg))
        {
            desired_angle_deg = (position_error_cm > 0.0f ?
                control->init_config.minimum_drive_angle_deg :
                -control->init_config.minimum_drive_angle_deg) *
                control->init_config.position_to_rod_direction;
        }
        if ((!returning) &&
            (BallAbs(position_error_cm) >
             control->init_config.sequence_tolerance_cm))
        {
            if (BallAbs(snapshot.measured_cm -
                        control->stall_reference_cm) >=
                control->init_config.stall_position_epsilon_cm)
            {
                control->stall_reference_cm = snapshot.measured_cm;
                control->stall_reference_tick = now_tick;
            }
            else if ((!control->disturbance_active) &&
                     ((now_tick - control->stall_reference_tick) >=
                      control->init_config.stall_detection_ms))
            {
                control->disturbance_active = true;
                control->disturbance_start_tick = now_tick;
                control->data.disturbance_count++;
                control->position_pid.reset(&control->position_pid);
            }

            if (control->disturbance_active)
            {
                if ((now_tick - control->disturbance_start_tick) <
                    control->init_config.disturbance_duration_ms)
                {
                    desired_angle_deg = (position_error_cm > 0.0f ?
                        control->init_config.disturbance_angle_deg :
                        -control->init_config.disturbance_angle_deg) *
                        control->init_config.position_to_rod_direction;
                }
                else
                {
                    control->disturbance_active = false;
                    control->stall_reference_cm = snapshot.measured_cm;
                    control->stall_reference_tick = now_tick;
                    control->position_pid.reset(&control->position_pid);
                }
            }
        }
        else
        {
            control->disturbance_active = false;
            control->stall_reference_cm = snapshot.measured_cm;
            control->stall_reference_tick = now_tick;
        }
        control->desired_angle_command_deg = desired_angle_deg;
        control->measurement_stale_start_tick = 0U;
        control->last_outer_measurement_tick =
            snapshot.measurement_tick;
        control->data.position_update_count++;
    }
    else if (measurement_stale)
    {
        if ((!returning) &&
            (control->data.state == BALL_CONTROL_ACTIVE))
        {
            uint32_t stale_elapsed_ms;
            uint32_t search_phase;
            float search_direction = 1.0f;

            if (control->measurement_stale_start_tick == 0U)
                control->measurement_stale_start_tick = now_tick;
            stale_elapsed_ms =
                now_tick - control->measurement_stale_start_tick;
            if (snapshot.measurement_valid &&
                ((snapshot.target_cm - snapshot.measured_cm) < 0.0f))
            {
                search_direction = -1.0f;
            }
            search_direction *=
                control->init_config.position_to_rod_direction;
            search_phase = stale_elapsed_ms /
                control->init_config.lost_search_step_ms;
            if ((search_phase & 1U) != 0U)
                search_direction = -search_direction;
            desired_angle_deg =
                stale_elapsed_ms <
                control->init_config.lost_search_timeout_ms ?
                search_direction *
                    control->init_config.lost_search_angle_deg :
                0.0f;
            control->desired_angle_command_deg = desired_angle_deg;
        }
        else
        {
            desired_angle_deg = 0.0f;
            control->desired_angle_command_deg = 0.0f;
        }
        control->position_pid.reset(&control->position_pid);
    }
    else
    {
        desired_angle_deg = control->desired_angle_command_deg;
    }
    motor_offset_deg = motor_data.position_deg - snapshot.motor_zero_deg;
    rod_angle_deg =
        BallMotorOffsetToRodAngle(control, motor_offset_deg);

    if (returning)
    {
        if (!control->return_leveling)
        {
            if ((!measurement_stale) &&
                (BallAbs(control->hold_target_cm -
                         snapshot.measured_cm) <=
                 control->init_config.return_ball_tolerance_cm))
            {
                if (!control->return_inside)
                {
                    control->return_inside = true;
                    control->return_inside_tick = now_tick;
                }
                else if ((now_tick - control->return_inside_tick) >=
                         control->init_config.return_ball_hold_ms)
                {
                    control->return_leveling = true;
                    control->return_forced_level = false;
                    control->return_inside = false;
                    control->position_pid.reset(&control->position_pid);
                    control->angle_pid.reset(&control->angle_pid);
                    desired_angle_deg = 0.0f;
                    control->desired_angle_command_deg = 0.0f;
                }
            }
            else
            {
                control->return_inside = false;
            }
        }
        else
        {
            if (BallAbs(rod_angle_deg) <=
                control->init_config.return_tolerance_deg)
            {
                if (!control->return_inside)
                {
                    control->return_inside = true;
                    control->return_inside_tick = now_tick;
                }
                else if ((now_tick - control->return_inside_tick) >=
                         control->init_config.return_hold_ms)
                {
                    BallMotorOff(control);
                    control->position_pid.reset(&control->position_pid);
                    control->angle_pid.reset(&control->angle_pid);
                    control->previous_speed_command_rpm = 0.0f;
                    control->requested_speed_command_rpm = 0.0f;
                    taskENTER_CRITICAL();
                    control->data.state = BALL_CONTROL_IDLE;
                    control->data.desired_rod_angle_deg = 0.0f;
                    control->data.rod_angle_deg = rod_angle_deg;
                    control->data.motor_speed_command_rpm = 0.0f;
                    taskEXIT_CRITICAL();
                    return;
                }
            }
            else
            {
                control->return_inside = false;
            }
        }

        if ((now_tick - control->return_start_tick) >=
            control->init_config.return_timeout_ms)
        {
            if (!control->return_leveling)
            {
                control->return_leveling = true;
                control->return_forced_level = true;
                control->return_inside = false;
                control->return_start_tick = now_tick;
                control->data.return_timeout_count++;
                control->position_pid.reset(&control->position_pid);
                control->angle_pid.reset(&control->angle_pid);
                desired_angle_deg = 0.0f;
                control->desired_angle_command_deg = 0.0f;
            }
            else
            {
                BallMotorOff(control);
                control->data.command_error_count++;
                control->data.motor_speed_command_rpm = 0.0f;
                control->data.state = BALL_CONTROL_FAULT;
                return;
            }
        }
    }
    new_motor_feedback =
        (motor_data.position_rx_tick != 0U) &&
        (motor_data.position_rx_tick !=
         control->last_inner_feedback_tick);
    if (new_motor_feedback)
    {
        inner_dt_s =
            control->last_inner_feedback_tick == 0U ?
            (float)control->init_config.motor_feedback_period_ms *
            0.001f :
            (float)(motor_data.position_rx_tick -
                    control->last_inner_feedback_tick) * 0.001f;
        if (inner_dt_s < 0.005f)
            inner_dt_s = 0.005f;
        if (inner_dt_s > 0.10f)
            inner_dt_s = 0.10f;
        speed_rpm = control->angle_pid.calculate(
            &control->angle_pid,
            rod_angle_deg,
            desired_angle_deg,
            inner_dt_s) * control->init_config.motor_direction;
        speed_rpm = BallClamp(
            speed_rpm,
            -control->init_config.maximum_motor_speed_rpm,
            control->init_config.maximum_motor_speed_rpm);
        maximum_step =
            control->init_config.motor_speed_slew_rpm_per_s *
            inner_dt_s;
        speed_rpm = BallClamp(
            speed_rpm,
            control->previous_speed_command_rpm - maximum_step,
            control->previous_speed_command_rpm + maximum_step);
        angle_error_deg = desired_angle_deg - rod_angle_deg;
        if ((BallAbs(angle_error_deg) >
             control->angle_pid.init_config.deadband) &&
            (BallAbs(speed_rpm) < 1.0f))
        {
            speed_rpm = angle_error_deg *
                        control->init_config.motor_direction > 0.0f ?
                        1.0f : -1.0f;
        }
        if (((motor_offset_deg <=
              control->init_config.minimum_motor_offset_deg) &&
             (speed_rpm < 0.0f)) ||
            ((motor_offset_deg >=
              control->init_config.maximum_motor_offset_deg) &&
             (speed_rpm > 0.0f)))
        {
            speed_rpm = 0.0f;
            control->requested_speed_command_rpm = 0.0f;
            if (control->previous_speed_command_rpm != 0.0f)
            {
                (void)control->init_config.motor->stop(
                    control->init_config.motor, false);
                control->previous_speed_command_rpm = 0.0f;
            }
        }
        control->requested_speed_command_rpm = speed_rpm;
        control->last_inner_feedback_tick =
            motor_data.position_rx_tick;
        control->data.angle_update_count++;
    }
    else
    {
        speed_rpm = control->requested_speed_command_rpm;
    }

    taskENTER_CRITICAL();
    control->data.predicted_cm = predicted_cm;
    control->data.desired_rod_angle_deg = desired_angle_deg;
    control->data.rod_angle_deg = rod_angle_deg;
    control->data.motor_speed_command_rpm = speed_rpm;
    control->data.motor_offset_deg = motor_offset_deg;
    control->data.measurement_stale = measurement_stale;
    if (measurement_stale)
        control->data.stale_count++;
    control->data.update_count++;
    taskEXIT_CRITICAL();

    if ((now_tick - control->last_command_tick) >=
        control->init_config.motor_command_period_ms)
    {
        control->last_command_tick = now_tick;
        if (BallSendSpeed(control, speed_rpm))
        {
            control->previous_speed_command_rpm = speed_rpm;
            control->data.command_count++;
        }
        else
        {
            control->data.command_error_count++;
        }
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

static bool BallSendSpeed(BallControl_t *control, float speed_rpm)
{
    return control->init_config.motor->set_speed(
        control->init_config.motor,
        speed_rpm,
        control->init_config.motor_acceleration,
        false);
}

static void BallMotorOff(BallControl_t *control)
{
    if ((control == NULL) ||
        (control->init_config.motor == NULL) ||
        (!control->init_config.motor->initialized))
    {
        return;
    }

    (void)control->init_config.motor->set_auto_return(
        control->init_config.motor, ZDT42_PARAM_POSITION, 0U);
    (void)control->init_config.motor->stop(
        control->init_config.motor, false);
    (void)control->init_config.motor->enable(
        control->init_config.motor, true, false);
}

static float BallMotorOffsetToRodAngle(const BallControl_t *control,
                                       float motor_offset_deg)
{
    motor_offset_deg = BallClamp(
        motor_offset_deg,
        control->init_config.minimum_motor_offset_deg,
        control->init_config.maximum_motor_offset_deg);
    float magnitude = BallAbs(motor_offset_deg);
    float rod_magnitude;
    float motor_sign = motor_offset_deg >= 0.0f ? 1.0f : -1.0f;

    if (motor_offset_deg >= 0.0f)
    {
        rod_magnitude = magnitude *
            (control->init_config.motor_to_rod_positive_linear +
             control->init_config.motor_to_rod_positive_quadratic *
                 magnitude);
    }
    else
    {
        rod_magnitude = magnitude *
            (control->init_config.motor_to_rod_negative_linear +
             control->init_config.motor_to_rod_negative_quadratic *
                 magnitude);
    }

    return rod_magnitude * motor_sign *
           control->init_config.motor_direction;
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
