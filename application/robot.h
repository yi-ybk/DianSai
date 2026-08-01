#pragma once

#include <pthread.h>
#include <stdint.h>

/** @brief 小球位置控制器的RAM在线调参区 */
typedef struct
{
    uint32_t enabled;                     /**< 0：关闭控制并回水平位，非0：启用控制 */
    float kp;                             /**< 比例增益(deg/px) */
    float ki;                             /**< 积分增益(deg/(px*s)) */
    float kd;                             /**< 速度反馈增益(deg/(px/s)) */
    float control_direction;              /**< 舵机控制方向，只使用正负号 */
    float level_angle_deg;                /**< 管道水平时的舵机角度 */
    float output_min_angle_deg;           /**< RAM软最小角度，仍受机械硬限幅保护 */
    float output_max_angle_deg;           /**< RAM软最大角度，仍受机械硬限幅保护 */
    float integral_limit_px_s;            /**< 积分绝对值上限(px*s) */
    float derivative_filter_tau_s;        /**< 测量速度低通滤波时间常数(s) */
    float deadband_px;                    /**< 位置误差死区(px) */
    float acceleration_ff_gain;           /**< 小车纵向加速度前馈增益(deg/(m/s^2))，允许用正负号修正方向 */
    float breakaway_angle_deg;            /**< 小球静止时的启动力前馈角度 */
    float breakaway_error_px;             /**< 启动力前馈的最小位置误差 */
    float breakaway_speed_px_s;            /**< 判定小球静止的速度阈值 */
    float maximum_slew_deg_per_s;          /**< 舵机指令最大变化速度 */
    uint32_t actual_acceleration_ff_enabled; /**< 1: use encoder wheel-speed acceleration */
    float actual_acceleration_filter_tau_s; /**< actual acceleration low-pass time constant */
    float actual_acceleration_limit_mps2;   /**< reject/clamp differentiated speed spikes */
    float actual_acceleration_deadband_mps2;/**< suppress steady-speed encoder noise */
} BallPidRamConfig_t;

/** @brief 小球位置控制器的RAM运行状态，供CCS在线观察 */
typedef struct
{
    float error_px;
    float integral_px_s;
    float measured_speed_px_s;
    float proportional_deg;
    float integral_deg;
    float derivative_deg;
    float acceleration_feedforward_deg;
    float breakaway_feedforward_deg;
    float mode3_movement_feedforward_deg;
    float unsaturated_angle_deg;
    float command_angle_deg;
    float dt_s;
    uint32_t update_count;
    uint32_t reset_count;
    float chassis_acceleration_mps2;       /**< 当前送入前馈的小车指令加速度(m/s^2) */
    float measured_chassis_speed_mps;
    float raw_chassis_acceleration_mps2;
    float acceleration_update_dt_s;
    uint32_t acceleration_update_count;
    uint32_t acceleration_reset_count;
} BallPidRamState_t;

typedef enum
{
    BALL_MODE3_IDLE = 0,
    BALL_MODE3_TO_POSITIVE,
    BALL_MODE3_TO_NEGATIVE,
    BALL_MODE3_FINAL_HOLD,
    BALL_MODE3_COMPLETE,
    BALL_MODE3_TIMEOUT,
    BALL_MODE3_ABORTED,
} BallMode3Phase_t;

/** Runtime-tunable parameters for requirement 3. */
typedef struct
{
    float center_px;
    float pixels_per_cm;
    float positive_pixel_direction;
    float endpoint_cm;
    float arrival_tolerance_cm;
    float movement_kp;
    float movement_ki;
    float movement_kd;
    float movement_derivative_filter_tau_s;
    float movement_deadband_px;
    float movement_feedforward_deg;
    float movement_ff_start_cm;
    float movement_ff_full_cm;
    float return_target_slew_cm_s;
    float arrival_speed_limit_cm_s;
    uint32_t arrival_hold_ms;
    uint32_t final_hold_ms;
    uint32_t timeout_ms;
} BallMode3RamConfig_t;

/** Observable state for the O -> +5 cm -> -5 cm sequence. */
typedef struct
{
    uint32_t phase;
    float center_px;
    float positive_target_px;
    float negative_target_px;
    float active_target_px;
    float error_px;
    float measured_speed_cm_s;
    uint32_t target_update_tick;
    uint32_t inside_since_tick;
    uint32_t final_hold_start_tick;
    uint32_t elapsed_ms;
    uint32_t completion_time_ms;
    uint32_t start_count;
    uint32_t complete_count;
    uint32_t timeout_count;
} BallMode3RamState_t;

extern volatile BallPidRamConfig_t ball_pid_ram;
extern volatile BallPidRamState_t ball_pid_state;
extern volatile BallMode3RamConfig_t ball_mode3_ram;
extern volatile BallMode3RamState_t ball_mode3_state;
extern volatile uint32_t ball_pid_reset_request;
extern volatile float ball_target;
extern volatile float ball_chassis_acceleration_mps2;
extern volatile float ball_real;
extern volatile float now_angel;

void robotInit(void);
