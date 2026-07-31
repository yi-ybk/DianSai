/**
 * @file ball_control.h
 * @brief Single-axis servo-driven ball-and-beam control framework.
 */
#pragma once

#include "motor_driver.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BALL_CONTROL_IDLE = 0,
    BALL_CONTROL_ARMING,
    BALL_CONTROL_ACTIVE,
    BALL_CONTROL_RETURNING,
    BALL_CONTROL_FAULT,
} BallControlState_t;

typedef enum
{
    BALL_PROFILE_HOLD = 0,
    BALL_PROFILE_REQUIREMENT3,
} BallControlProfile_t;

typedef enum
{
    BALL_SEQUENCE_HOLD = 0,
    BALL_SEQUENCE_TO_POSITIVE,
    BALL_SEQUENCE_TO_NEGATIVE,
    BALL_SEQUENCE_COMPLETE,
} BallControlSequence_t;

typedef struct
{
    Motor_t *motor;
    PidInitConfig_t position_pid;
    float motor_to_rod_ratio;
    float motor_direction;
    float servo_center_angle_deg;
    float maximum_target_cm;
    float maximum_rod_angle_deg;
    float maximum_servo_slew_deg_per_s;
    float measurement_velocity_filter_s;
    float prediction_horizon_s;
    float sequence_tolerance_cm;
    uint16_t control_period_ms;
    uint16_t motor_command_period_ms;
    uint16_t measurement_timeout_ms;
    uint16_t sequence_hold_ms;
    uint16_t stall_detection_ms;
    uint16_t disturbance_duration_ms;
    uint16_t lost_search_step_ms;
    uint16_t lost_search_timeout_ms;
    uint16_t return_ball_hold_ms;
    uint16_t return_hold_ms;
    uint16_t return_timeout_ms;
} BallControlInitConfig_t;

typedef struct
{
    BallControlState_t state;
    BallControlProfile_t profile;
    BallControlSequence_t sequence;
    float target_cm;
    float measured_cm;
    float measured_velocity_cm_s;
    float predicted_cm;
    float desired_rod_angle_deg;
    float rod_angle_deg;
    float servo_angle_command_deg;
    uint32_t start_tick;
    uint32_t completion_time_ms;
    uint32_t measurement_tick;
    uint32_t update_count;
    uint32_t position_update_count;
    uint32_t angle_update_count;
    uint32_t command_count;
    uint32_t command_error_count;
    uint32_t stale_count;
    uint32_t disturbance_count;
    uint32_t return_timeout_count;
    bool measurement_valid;
    bool measurement_stale;
    bool sequence_complete;
} BallControlData_t;

typedef struct BallControl BallControl_t;

struct BallControl
{
    bool initialized;
    BallControlInitConfig_t init_config;
    BallControlData_t data;
    Pid_t position_pid;
    float hold_target_cm;
    float positive_target_cm;
    float negative_target_cm;
    float previous_measurement_cm;
    float previous_servo_angle_deg;
    uint32_t previous_measurement_tick;
    uint32_t last_outer_measurement_tick;
    uint32_t last_inner_feedback_tick;
    uint32_t last_update_tick;
    uint32_t last_command_tick;
    uint32_t sequence_inside_tick;
    uint32_t stall_reference_tick;
    uint32_t disturbance_start_tick;
    uint32_t measurement_stale_start_tick;
    uint32_t return_start_tick;
    uint32_t return_inside_tick;
    bool sequence_inside;
    bool disturbance_active;
    bool return_inside;
    bool return_leveling;
    bool return_forced_level;

    bool (*init)(BallControl_t *control,
                 const BallControlInitConfig_t *config);
    bool (*start)(BallControl_t *control,
                  BallControlProfile_t profile,
                  float hold_target_cm,
                  float positive_target_cm,
                  float negative_target_cm,
                  uint32_t now_tick);
    void (*stop)(BallControl_t *control);
    void (*set_target)(BallControl_t *control, float target_cm);
    void (*set_measurement)(BallControl_t *control,
                            float position_cm,
                            uint32_t measurement_tick);
    void (*update)(BallControl_t *control, uint32_t now_tick);
    void (*get_data)(BallControl_t *control, BallControlData_t *data);
};

bool BallControlInit(BallControl_t *control,
                     const BallControlInitConfig_t *config);
bool BallControlStart(BallControl_t *control,
                      BallControlProfile_t profile,
                      float hold_target_cm,
                      float positive_target_cm,
                      float negative_target_cm,
                      uint32_t now_tick);
void BallControlStop(BallControl_t *control);
void BallControlSetTarget(BallControl_t *control, float target_cm);
void BallControlSetMeasurement(BallControl_t *control,
                               float position_cm,
                               uint32_t measurement_tick);
void BallControlUpdate(BallControl_t *control, uint32_t now_tick);
void BallControlGetData(BallControl_t *control, BallControlData_t *data);

#define BALL_CONTROL_OBJECT_DEFAULT                 \
    .init            = BallControlInit,             \
    .start           = BallControlStart,            \
    .stop            = BallControlStop,             \
    .set_target      = BallControlSetTarget,        \
    .set_measurement = BallControlSetMeasurement,   \
    .update          = BallControlUpdate,           \
    .get_data        = BallControlGetData
