#include "robot.h"
#include "cmsis_os.h"
#include "ti_msp_dl_config.h"

#include "led_driver.h"

#include "oled_driver.h"

#include "key_driver.h"

#include "gray.h"
#include "track.h"

#include "encoder.h"
#include "motor_driver.h"
#include "pid.h"
#include "chassis.h"
#include "slaver.h"
#include <string.h>

/************ 函数声明 **************/
static void chassisControlTask(void *argument);
static void oledTask(void *argument);
static void trackTask(void *argument);
static void ballHostControlTask(void *argument);

static void key1ProcessEvents(void);
static void keyA07HandleInterrupt(void);
static void oledDrawDefaultPage(void);
static void oledDrawBallPage(void);
static bool trackModeIsSupported(uint8_t selected_mode);
static bool trackModeUsesStableProfile(uint8_t selected_mode);
static bool trackModeUsesBallLapProfile(uint8_t selected_mode);
static bool trackFinishLineDetected(float distance_m);
static void trackRunRequirement2(uint32_t now_tick, float dt_s);
static void trackRunRequirement4(uint32_t now_tick, float dt_s);
static void trackRunRequirement5Or6(uint32_t now_tick, float dt_s);
static void trackStart(uint32_t start_tick);
static void trackStop(uint32_t stop_tick);
static void trackDebugReset(void);
static void trackDebugCapture(uint32_t now_tick);
static void trackDebugStop(uint32_t stop_tick);
static float trackLimitForwardSpeed(float requested_speed_mps);
static float trackSmoothForwardSpeed(float target_speed_mps, float dt_s);
static float trackWrapAngleRad(float angle_rad);
static bool ballPositionPixelValid(float position_px);
static void trackInit(void);
static void ledInit(void);
static void oledInit(void);
static void keyInit(void);
static void grayInit(void);
static void encoderInit(void);
static void motorInit(void);
static void pidInit(void);
static void wheelInit(void);
static void chassisInit(void);
static void gpioInterruptDispatch(GPIO_TypeDef *GPIOx);
static float ballPidUpdate(float target_px,
                           float measured_px,
                           float chassis_acceleration_mps2,
                           float dt_s);
static void ballPidReset(void);
static void ballPidGetConfig(BallPidRamConfig_t *config);
static float ballPidClamp(float value, float minimum, float maximum);
static float ballPidAbs(float value);
static float ballPidSanitize(float value,
                             float fallback,
                             float minimum,
                             float maximum);

/***********************************/

/******** 线程句柄和属性 ********/
static StaticTask_t chassisControlTaskControlBlock;
static StackType_t chassisControlTaskStack[512];
osThreadId_t chassisControlTaskHandle;
const osThreadAttr_t chassisControlTask_attributes = {
  .name = "chassisControlTask",
  .cb_mem = &chassisControlTaskControlBlock,
  .cb_size = sizeof(chassisControlTaskControlBlock),
  .stack_mem = chassisControlTaskStack,
  .stack_size = sizeof(chassisControlTaskStack),
  .priority = (osPriority_t) osPriorityAboveNormal,
};

static StaticTask_t oledTaskControlBlock;
static StackType_t oledTaskStack[512];
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .cb_mem = &oledTaskControlBlock,
  .cb_size = sizeof(oledTaskControlBlock),
  .stack_mem = oledTaskStack,
  .stack_size = sizeof(oledTaskStack),
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t trackTaskHandle;
static StaticTask_t trackTaskControlBlock;
static StackType_t trackTaskStack[256];
const osThreadAttr_t trackTask_attributes = {
  .name = "trackTask",
  .cb_mem = &trackTaskControlBlock,
  .cb_size = sizeof(trackTaskControlBlock),
  .stack_mem = trackTaskStack,
  .stack_size = sizeof(trackTaskStack),
  .priority = (osPriority_t) osPriorityHigh,
};

static StaticTask_t ballHostControlTaskControlBlock;
static StackType_t ballHostControlTaskStack[256];
osThreadId_t ballHostControlTaskHandle;
const osThreadAttr_t ballHostControlTask_attributes = {
  .name = "ballControlTask",
  .cb_mem = &ballHostControlTaskControlBlock,
  .cb_size = sizeof(ballHostControlTaskControlBlock),
  .stack_mem = ballHostControlTaskStack,
  .stack_size = sizeof(ballHostControlTaskStack),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/*******************************/

/*********** 对象实例 ***********/
Led_t led_green = { LED_OBJECT_DEFAULT };
Led_t led_red   = { LED_OBJECT_DEFAULT };
Led_t led_blue  = { LED_OBJECT_DEFAULT };

Oled_t oled = { OLED_OBJECT_DEFAULT };

Key_t key1 = { KEY_OBJECT_DEFAULT };
Key_t key2 = { KEY_OBJECT_DEFAULT };

Gray_t gray = { GRAY_OBJECT_DEFAULT };
Track_t track = { TRACK_OBJECT_DEFAULT };

Encoder_t encoder_left  = { ENCODER_OBJECT_DEFAULT };
Encoder_t encoder_right = { ENCODER_OBJECT_DEFAULT };

Motor_t motor_left  = { MOTOR_OBJECT_DEFAULT };
Motor_t motor_right = { MOTOR_OBJECT_DEFAULT };
Motor_t motor_ball_servo = { MOTOR_OBJECT_DEFAULT };

TIM_HandleTypeDef htim_motor_pwm = {
    .Instance = PWM_MOTOR_INST,
    .Init = {
        .Prescaler = 0U,
        .Period = 3999U,
    },
    .clock_hz = PWM_MOTOR_INST_CLK_FREQ,
};

TIM_HandleTypeDef htim_ball_servo_pwm = {
    .Instance = PWM_BALL_SERVO_INST,
    .Init = {
        .Prescaler = 0U,
        .Period = 39999U,
    },
    .clock_hz = PWM_BALL_SERVO_INST_CLK_FREQ,
};

static UART_HandleTypeDef huart_hc12 = {
    .Instance = UART_HC12_INST,
};

Pid_t wheel_left_pid  = { PID_OBJECT_DEFAULT };
Pid_t wheel_right_pid = { PID_OBJECT_DEFAULT };

Wheel_t wheel_left  = { WHEEL_OBJECT_DEFAULT };
Wheel_t wheel_right = { WHEEL_OBJECT_DEFAULT };

Chassis_t chassis = { CHASSIS_OBJECT_DEFAULT };

Slaver_t ball_position_receiver = { SLAVER_OBJECT_DEFAULT };
static SlaverSimpleFloatProtocol_t ball_position_protocol;

volatile float ball_target = 150.0f;
volatile float ball_chassis_acceleration_mps2 = 0.0f;
volatile float ball_real = 0.0f;

#define ROBOT_MODE_REQUIREMENT_3          3U
#define ROBOT_MODE_REQUIREMENT_2          2U
#define ROBOT_MODE_REQUIREMENT_4          4U
#define ROBOT_MODE_REQUIREMENT_5          5U
#define ROBOT_MODE_REQUIREMENT_6          6U
#define BALL_SERVO_CENTER_ANGLE_DEG       55.0f
#define TRACK_USE_SMALL_TEST_MAP          0U
#if TRACK_USE_SMALL_TEST_MAP
#define TRACK_LAP_DISTANCE_M              3.75f
#define TRACK_FINISH_ARM_DISTANCE_M       3.50f
#else
#define TRACK_LAP_DISTANCE_M              (3.0f + 3.14159265f)
#define TRACK_FINISH_ARM_DISTANCE_M       5.25f
#endif
#define TRACK_POSITION_TOLERANCE_M        0.02f
#define TRACK_POSITION_KP                 1.0f
#define TRACK_FINISH_LINE_LEFT_SUPPORT_MASK  0x0CU
#define TRACK_FINISH_LINE_RIGHT_SUPPORT_MASK 0x30U
#define TRACK_FINISH_LINE_ARM_TIME_MS     2000U
#define TRACK_FINISH_LINE_CONFIRM_SAMPLES 6U
#define TRACK_FINISH_LINE_PRIMARY_SCORE   3U
#define TRACK_FINISH_LINE_SUPPORT_SCORE   1U
#define TRACK_FINISH_LINE_SUPPORT_ONLY_SCORE 12U
#define TRACK_FINISH_LINE_MAX_SCORE       12U
#define TRACK_FINISH_ALIGN_HEADING_KP        1.50f
#define TRACK_FINISH_ALIGN_HEADING_TOLERANCE_RAD 0.026f
#define TRACK_FINISH_ALIGN_MAX_ROTATION_RAD  0.262f
#define TRACK_FINISH_ALIGN_MAX_TIME_MS     1500U
#define TRACK_FINISH_ALIGN_CONFIRM_SAMPLES 20U
#define TRACK_FINISH_ALIGN_MIN_TURN_SPEED  0.10f
#define TRACK_FINISH_ALIGN_MAX_TURN_SPEED  0.35f
#define TRACK_FINISH_REVERSE_DISTANCE_M    0.020f
#define TRACK_FINISH_REVERSE_SPEED_MPS     0.10f
#define TRACK_FINISH_REVERSE_MAX_TIME_MS   800U
#define TRACK_MAX_RUN_TIME_MS             20000U
#define REQUIREMENT4_AB_DISTANCE_M        1.5f
#define REQUIREMENT4_FORWARD_SPEED_MPS    0.35f
#define REQUIREMENT4_MAX_RUN_TIME_MS      8000U
#define REQUIREMENT4_CURVE_ARM_DISTANCE_M 0.9f
#define REQUIREMENT4_CURVE_ERROR_THRESHOLD 2.0f
#define REQUIREMENT4_CURVE_CONFIRM_SAMPLES 4U
#define KEY_A07_DEBOUNCE_MS               50U
#define OLED_COUNTER_INTERVAL_MS           1000U
#define TRACK_CONTROL_DT_S                 0.005f
#define TRACK_START_REQUEST_MAX_AGE_MS        50U
#define TRACK_CORNER_EXIT_HOLD_MS          350U
#define TRACK_FORWARD_ACCEL_MPS2           1.10f
#define TRACK_FORWARD_DECEL_MPS2           3.50f
#define TRACK_AGGRESSIVE_MAX_TURN_SPEED     0.72f
#define TRACK_RECOVERY_MAX_TURN_SPEED       0.72f
#define TRACK_LARGE_ERROR_TURN_GAIN          1.25f
#define TRACK_MODE2_TRANSIENT_LOSS_TIME_S     0.16f
#define TRACK_MODE2_TRANSIENT_LOSS_SPEED_SCALE 0.90f
#define TRACK_MODE2_LOST_LINE_SPEED_SCALE    0.45f
#define TRACK_MODE2_OUTER_SENSOR_SPEED_SCALE 0.30f
#define TRACK_MODE2_LARGE_ERROR_SPEED_SCALE  0.42f
#define TRACK_MODE2_CORNER_HOLD_SPEED_SCALE  0.48f
#define TRACK_MODE2_FINISH_APPROACH_SPEED_MPS 0.16f
#define TRACK_STABLE_BASE_SPEED_MPS         0.45f
#define TRACK_STABLE_FORWARD_ACCEL_MPS2     0.60f
#define TRACK_STABLE_FORWARD_DECEL_MPS2     0.90f
#define TRACK_STABLE_SOFT_START_MS          2000U
#define TRACK_STABLE_MAX_RUN_TIME_MS       30000U
#define TRACK_BALL_LAP_BASE_SPEED_MPS       0.30f
#define TRACK_BALL_LAP_MAX_TURN_SPEED       0.75f
#define TRACK_BALL_LAP_TURN_GAIN             1.20f
#define TRACK_BALL_LAP_FORWARD_ACCEL_MPS2   0.25f
#define TRACK_BALL_LAP_FORWARD_DECEL_MPS2   0.25f
#define BALL_HOST_FRAME_HEADER              0xA5U
#define BALL_HOST_FRAME_TAIL                0x5AU
#define BALL_HOST_FRAME_LENGTH              7U
#define BALL_HOST_RX_BUFFER_SIZE             BALL_HOST_FRAME_LENGTH
#define BALL_HOST_TASK_PERIOD_MS             10U
#define BALL_POSITION_MIN_PX                  0.0f
#define BALL_POSITION_MAX_PX                300.0f
#define BALL_HOST_INVALID_POSITION           -1.0f
#define BALL_MODE3_START_TOLERANCE_PX         60.0f
#define BALL_MODE3_START_MAX_AGE_MS          500U
/*
TRACK_LAP_DISTANCE_M：跑一圈的目标距离，当前约 6.1416m。
TRACK_FINISH_ARM_DISTANCE_M：接近一圈末段后，才允许识别 A 点终点黑线，避免刚启动就误判。
TRACK_POSITION_TOLERANCE_M：停车位置允许误差，当前 0.02m，即题目要求的 2cm。
TRACK_POSITION_KP：位置环 P 参数。增大后接近终点时速度更高，但容易冲过；减小后减速更早、更平稳。
TRACK_FINISH_LINE_MIN_BLACK_COUNT
间，当前 20000ms。
REQUIREMENT4_AB_DISTANCE_M：要求 4 从 A 到 B 的目标里程，当前 1.5m。
REQUIREMENT4_FORWARD_SPEED_MPS：要求 4 的巡线前进速度，当前 0.35m/s。
REQUIREMENT4_MAX_RUN_TIME_MS：要求 4 的 AB 最大运行时间，当前 8000ms。
*/
volatile uint8_t mode = ROBOT_MODE_REQUIREMENT_2;
volatile uint32_t oled_refresh_error_count = 0U;
volatile uint32_t oled_recovery_count = 0U;
volatile uint32_t ball_test_error = 0U;
volatile uint32_t ball_host_init_error = 0U;
volatile uint32_t ball_host_valid_position_count = 0U;
volatile uint32_t ball_host_invalid_position_count = 0U;
volatile uint32_t ball_host_rejected_position_count = 0U;
volatile uint32_t ball_host_integer_position_count = 0U;
volatile uint32_t ball_host_fractional_position_count = 0U;
volatile uint32_t ball_host_period_min_ms = 0U;
volatile uint32_t ball_host_period_max_ms = 0U;
volatile float ball_host_period_ema_ms = 0.0f;
volatile uint32_t ball_host_last_frame_tick = 0U;
static uint32_t oled_default_counter = 0U;
static uint8_t requirement4_curve_detect_count = 0U;
volatile uint8_t track_finish_line_detect_count = 0U;
volatile uint8_t track_finish_last_mask = 0U;
volatile uint8_t track_finish_last_raw_count = 0U;
volatile uint32_t track_finish_pattern_hit_count = 0U;
volatile uint32_t track_finish_trigger_count = 0U;
volatile uint32_t track_finish_three_channel_hit_count = 0U;
volatile uint32_t track_finish_two_channel_hit_count = 0U;
volatile float track_finish_trigger_distance_m = 0.0f;
volatile float track_stop_distance_m = 0.0f;
volatile uint8_t track_stop_reason = 0U;
volatile bool track_finish_three_channel_seen = false;
volatile bool track_finish_align_active = false;
volatile uint8_t track_finish_align_settle_count = 0U;
volatile float track_finish_align_start_turn_angle_rad = 0.0f;
volatile float track_finish_align_error_rad = 0.0f;
volatile uint32_t track_finish_align_start_tick = 0U;
volatile bool track_finish_reverse_active = false;
volatile float track_finish_reverse_start_distance_m = 0.0f;
volatile uint32_t track_finish_reverse_start_tick = 0U;

static volatile bool ball_menu_active = false;
static volatile bool track_running = false;
static volatile bool track_start_requested = false;
static volatile bool track_stop_requested = false;
static volatile uint32_t track_start_request_tick = 0U;
static volatile uint32_t track_start_tick = 0U;
static volatile uint32_t track_elapsed_ms = 0U;
static volatile uint32_t track_total_time_ms = 0U;
static volatile bool track_timer_stopped = true;
static uint32_t track_slow_until_tick = 0U;
static float track_forward_speed_command = 0.0f;
static bool key1_sample_pressed = false;
static bool key1_stable_pressed = false;
static bool key1_long_press_handled = false;
static uint32_t key1_sample_change_tick = 0U;
static uint32_t key1_press_tick = 0U;
static volatile uint32_t key_a07_last_toggle_tick = 0U;
/*******************************/
#define BALL_SERVO_HARD_MIN_ANGLE_DEG        48.0f
#define BALL_SERVO_HARD_MAX_ANGLE_DEG        65.0f
#define BALL_CAMERA_TIMEOUT_MS              200U
#define BALL_PID_DEFAULT_DT_S                 0.02f
#define BALL_PID_MIN_DT_S                     0.005f
#define BALL_PID_MAX_DT_S                     0.20f
#define BALL_PID_MAX_ACCELERATION_FF_DEG       2.0f

volatile BallPidRamConfig_t ball_pid_ram = {
    .enabled = 0U,
    .kp = 0.45f,
    .ki = 0.01f,
    .kd = 0.11f,
    .control_direction = 1.0f,
    .level_angle_deg = BALL_SERVO_CENTER_ANGLE_DEG,
    .output_min_angle_deg = BALL_SERVO_HARD_MIN_ANGLE_DEG,
    .output_max_angle_deg = BALL_SERVO_HARD_MAX_ANGLE_DEG,
    .integral_limit_px_s = 30.0f,
    .derivative_filter_tau_s = 0.05f,
    .deadband_px = 1.5f,
    .acceleration_ff_gain = 2.0f,
    .breakaway_angle_deg = 0.1f,
    .breakaway_error_px = 4.0f,
    .breakaway_speed_px_s = 10.0f,
    .maximum_slew_deg_per_s = 60.0f,
};
volatile BallPidRamState_t ball_pid_state = {0};
volatile TrackDebugState_t track_debug_state = {0};
volatile uint32_t ball_pid_reset_request = 0U;
volatile float now_angel = BALL_SERVO_CENTER_ANGLE_DEG;

static float ball_pid_integral_px_s = 0.0f;
static float ball_pid_previous_measurement_px = 0.0f;
static float ball_pid_filtered_speed_px_s = 0.0f;
static bool ball_pid_has_measurement = false;
void robotInit(void)
{
    /* 初始化机器人相关的硬件和软件组件 */
    ledInit();
    oledInit();
    keyInit();
    grayInit();
    trackInit();
    encoderInit();
    motorInit();
    pidInit();
    wheelInit();
    chassisInit();
   
    chassisControlTaskHandle = osThreadNew(chassisControlTask, NULL, &chassisControlTask_attributes);
    oledTaskHandle = osThreadNew(oledTask, NULL, &oledTask_attributes);
    trackTaskHandle = osThreadNew(trackTask, NULL, &trackTask_attributes);
    ballHostControlTaskHandle =
        osThreadNew(ballHostControlTask, NULL,
                    &ballHostControlTask_attributes);
    // configASSERT(chassisControlTaskHandle != NULL);
    // configASSERT(oledTaskHandle != NULL);
    // configASSERT(trackTaskHandle != NULL);
    // configASSERT(ballHostControlTaskHandle != NULL);
}

static void chassisControlTask(void *argument)
{
    uint32_t last_processed_camera_tick = 0U;
    bool camera_control_active = false;

    (void)argument;
    /* 平衡闭环独立于循迹模式；上电后首个有效相机坐标到达即开始控制。 */
    motor_ball_servo.set_angle(&motor_ball_servo,
                               BALL_SERVO_CENTER_ANGLE_DEG);
    while (1)
    {
        BallPidRamConfig_t config;
        float camera_position_px;
        float target_position_px;
        float chassis_acceleration_mps2;
        uint32_t camera_frame_tick;
        uint32_t now_tick;
        bool camera_data_usable;

        taskENTER_CRITICAL();
        camera_position_px = ball_real;
        target_position_px = ball_target;
        chassis_acceleration_mps2 = ball_chassis_acceleration_mps2;
        camera_frame_tick = ball_host_last_frame_tick;
        taskEXIT_CRITICAL();

        ballPidGetConfig(&config);
        now_tick = HAL_GetTick();
        camera_data_usable =
            (config.enabled != 0U) &&
            (camera_frame_tick != 0U) &&
            ((now_tick - camera_frame_tick) <= BALL_CAMERA_TIMEOUT_MS) &&
            (camera_position_px != BALL_HOST_INVALID_POSITION) &&
            ballPositionPixelValid(camera_position_px) &&
            ballPositionPixelValid(target_position_px);

        if (ball_pid_reset_request != 0U)
        {
            ballPidReset();
            last_processed_camera_tick = 0U;
        }

        if (camera_data_usable &&
            (camera_frame_tick != last_processed_camera_tick))
        {
            float frame_dt_s = BALL_PID_DEFAULT_DT_S;

            if (last_processed_camera_tick != 0U)
            {
                frame_dt_s =
                    (float)(camera_frame_tick - last_processed_camera_tick) *
                    0.001f;
                frame_dt_s = ballPidClamp(frame_dt_s,
                                          BALL_PID_MIN_DT_S,
                                          BALL_PID_MAX_DT_S);
            }

            /* camera_position_px 就是相机识别到的像素坐标 */
            now_angel = ballPidUpdate(target_position_px,
                                      camera_position_px,
                                      chassis_acceleration_mps2,
                                      frame_dt_s);
            motor_ball_servo.set_angle(&motor_ball_servo, now_angel);
            last_processed_camera_tick = camera_frame_tick;
            camera_control_active = true;
        }
        else if (!camera_data_usable)
        {
            float level_angle_deg = ballPidClamp(
                config.level_angle_deg,
                BALL_SERVO_HARD_MIN_ANGLE_DEG,
                BALL_SERVO_HARD_MAX_ANGLE_DEG);

            if (camera_control_active)
                ballPidReset();
            if (ballPidAbs(now_angel - level_angle_deg) > 0.001f)
            {
                now_angel = level_angle_deg;
                motor_ball_servo.set_angle(&motor_ball_servo, now_angel);
            }
            last_processed_camera_tick = 0U;
            camera_control_active = false;
        }
        chassis.update(&chassis,0.005f);
        osDelay(5);
    }
}

static float ballPidUpdate(float target_px,
                           float measured_px,
                           float chassis_acceleration_mps2,
                           float dt_s)
{
    BallPidRamConfig_t config;
    float error_px;
    float raw_speed_px_s = 0.0f;
    float filter_alpha;
    float candidate_integral_px_s;
    float proportional_deg;
    float integral_deg;
    float derivative_deg;
    float acceleration_feedforward_deg;
    float breakaway_feedforward_deg = 0.0f;
    float unsaturated_angle_deg;
    float integral_command_change_deg;
    float command_angle_deg;
    float maximum_step_deg;

    ballPidGetConfig(&config);
    dt_s = ballPidSanitize(dt_s,
                           BALL_PID_DEFAULT_DT_S,
                           BALL_PID_MIN_DT_S,
                           BALL_PID_MAX_DT_S);
    chassis_acceleration_mps2 = ballPidSanitize(
        chassis_acceleration_mps2, 0.0f, -5.0f, 5.0f);

    error_px = target_px - measured_px;
    if (ballPidAbs(error_px) <= config.deadband_px)
        error_px = 0.0f;

    if (ball_pid_has_measurement)
        raw_speed_px_s =
            (measured_px - ball_pid_previous_measurement_px) / dt_s;
    ball_pid_previous_measurement_px = measured_px;
    ball_pid_has_measurement = true;

    if (config.derivative_filter_tau_s <= 0.0f)
        filter_alpha = 1.0f;
    else
        filter_alpha = dt_s /
            (config.derivative_filter_tau_s + dt_s);
    ball_pid_filtered_speed_px_s += filter_alpha *
        (raw_speed_px_s - ball_pid_filtered_speed_px_s);

    candidate_integral_px_s = ball_pid_integral_px_s;
    if (config.ki > 0.0f)
    {
        candidate_integral_px_s += error_px * dt_s;
        candidate_integral_px_s = ballPidClamp(
            candidate_integral_px_s,
            -config.integral_limit_px_s,
            config.integral_limit_px_s);
    }
    else
    {
        candidate_integral_px_s = 0.0f;
    }

    proportional_deg = config.kp * error_px;
    integral_deg = config.ki * candidate_integral_px_s;
    derivative_deg = -config.kd * ball_pid_filtered_speed_px_s;
    acceleration_feedforward_deg = ballPidClamp(
        config.acceleration_ff_gain * chassis_acceleration_mps2,
        -BALL_PID_MAX_ACCELERATION_FF_DEG,
        BALL_PID_MAX_ACCELERATION_FF_DEG);

    if ((config.breakaway_angle_deg > 0.0f) &&
        (ballPidAbs(error_px) >= config.breakaway_error_px) &&
        (ballPidAbs(ball_pid_filtered_speed_px_s) <=
         config.breakaway_speed_px_s))
    {
        breakaway_feedforward_deg = error_px > 0.0f ?
            config.breakaway_angle_deg : -config.breakaway_angle_deg;
    }

    unsaturated_angle_deg = config.level_angle_deg +
        config.control_direction *
        (proportional_deg + integral_deg + derivative_deg +
         acceleration_feedforward_deg + breakaway_feedforward_deg);

    integral_command_change_deg = config.control_direction * config.ki *
        (candidate_integral_px_s - ball_pid_integral_px_s);
    if (!(((unsaturated_angle_deg > config.output_max_angle_deg) &&
           (integral_command_change_deg > 0.0f)) ||
          ((unsaturated_angle_deg < config.output_min_angle_deg) &&
           (integral_command_change_deg < 0.0f))))
    {
        ball_pid_integral_px_s = candidate_integral_px_s;
    }

    integral_deg = config.ki * ball_pid_integral_px_s;
    unsaturated_angle_deg = config.level_angle_deg +
        config.control_direction *
        (proportional_deg + integral_deg + derivative_deg +
         acceleration_feedforward_deg + breakaway_feedforward_deg);
    command_angle_deg = ballPidClamp(unsaturated_angle_deg,
                                     config.output_min_angle_deg,
                                     config.output_max_angle_deg);

    maximum_step_deg = config.maximum_slew_deg_per_s * dt_s;
    command_angle_deg = ballPidClamp(command_angle_deg,
                                     now_angel - maximum_step_deg,
                                     now_angel + maximum_step_deg);
    command_angle_deg = ballPidClamp(command_angle_deg,
                                     BALL_SERVO_HARD_MIN_ANGLE_DEG,
                                     BALL_SERVO_HARD_MAX_ANGLE_DEG);

    ball_pid_state.error_px = error_px;
    ball_pid_state.integral_px_s = ball_pid_integral_px_s;
    ball_pid_state.measured_speed_px_s = ball_pid_filtered_speed_px_s;
    ball_pid_state.proportional_deg = proportional_deg;
    ball_pid_state.integral_deg = integral_deg;
    ball_pid_state.derivative_deg = derivative_deg;
    ball_pid_state.acceleration_feedforward_deg =
        acceleration_feedforward_deg;
    ball_pid_state.breakaway_feedforward_deg =
        breakaway_feedforward_deg;
    ball_pid_state.unsaturated_angle_deg = unsaturated_angle_deg;
    ball_pid_state.command_angle_deg = command_angle_deg;
    ball_pid_state.dt_s = dt_s;
    ball_pid_state.chassis_acceleration_mps2 =
        chassis_acceleration_mps2;
    ball_pid_state.update_count++;

    return command_angle_deg;
}

static void ballPidReset(void)
{
    ball_pid_integral_px_s = 0.0f;
    ball_pid_previous_measurement_px = 0.0f;
    ball_pid_filtered_speed_px_s = 0.0f;
    ball_pid_has_measurement = false;
    ball_pid_state.error_px = 0.0f;
    ball_pid_state.integral_px_s = 0.0f;
    ball_pid_state.measured_speed_px_s = 0.0f;
    ball_pid_state.proportional_deg = 0.0f;
    ball_pid_state.integral_deg = 0.0f;
    ball_pid_state.derivative_deg = 0.0f;
    ball_pid_state.acceleration_feedforward_deg = 0.0f;
    ball_pid_state.breakaway_feedforward_deg = 0.0f;
    ball_pid_state.unsaturated_angle_deg = now_angel;
    ball_pid_state.command_angle_deg = now_angel;
    ball_pid_state.dt_s = 0.0f;
    ball_pid_state.chassis_acceleration_mps2 = 0.0f;
    ball_pid_state.reset_count++;
    ball_pid_reset_request = 0U;
}

static void ballPidGetConfig(BallPidRamConfig_t *config)
{
    float direction;

    if (config == NULL)
        return;

    config->enabled = ball_pid_ram.enabled;
    config->kp = ballPidSanitize(ball_pid_ram.kp, 0.03f, 0.0f, 1.0f);
    config->ki = ballPidSanitize(ball_pid_ram.ki, 0.0f, 0.0f, 1.0f);
    config->kd = ballPidSanitize(ball_pid_ram.kd, 0.005f, 0.0f, 1.0f);
    direction = ballPidSanitize(ball_pid_ram.control_direction,
                                1.0f, -1.0f, 1.0f);
    config->control_direction = direction < 0.0f ? -1.0f : 1.0f;
    config->level_angle_deg = ballPidSanitize(
        ball_pid_ram.level_angle_deg,
        BALL_SERVO_CENTER_ANGLE_DEG,
        BALL_SERVO_HARD_MIN_ANGLE_DEG,
        BALL_SERVO_HARD_MAX_ANGLE_DEG);
    config->output_min_angle_deg = ballPidSanitize(
        ball_pid_ram.output_min_angle_deg,
        BALL_SERVO_HARD_MIN_ANGLE_DEG,
        BALL_SERVO_HARD_MIN_ANGLE_DEG,
        BALL_SERVO_HARD_MAX_ANGLE_DEG);
    config->output_max_angle_deg = ballPidSanitize(
        ball_pid_ram.output_max_angle_deg,
        BALL_SERVO_HARD_MAX_ANGLE_DEG,
        BALL_SERVO_HARD_MIN_ANGLE_DEG,
        BALL_SERVO_HARD_MAX_ANGLE_DEG);
    if (config->output_min_angle_deg > config->output_max_angle_deg)
    {
        config->output_min_angle_deg = BALL_SERVO_HARD_MIN_ANGLE_DEG;
        config->output_max_angle_deg = BALL_SERVO_HARD_MAX_ANGLE_DEG;
    }
    config->integral_limit_px_s = ballPidSanitize(
        ball_pid_ram.integral_limit_px_s, 30.0f, 0.0f, 10000.0f);
    config->derivative_filter_tau_s = ballPidSanitize(
        ball_pid_ram.derivative_filter_tau_s, 0.05f, 0.0f, 2.0f);
    config->deadband_px = ballPidSanitize(
        ball_pid_ram.deadband_px, 1.5f, 0.0f, 50.0f);
    config->acceleration_ff_gain = ballPidSanitize(
        ball_pid_ram.acceleration_ff_gain, 2.0f, -10.0f, 10.0f);
    config->breakaway_angle_deg = ballPidSanitize(
        ball_pid_ram.breakaway_angle_deg, 0.0f, 0.0f, 3.0f);
    config->breakaway_error_px = ballPidSanitize(
        ball_pid_ram.breakaway_error_px, 4.0f, 0.0f, 100.0f);
    config->breakaway_speed_px_s = ballPidSanitize(
        ball_pid_ram.breakaway_speed_px_s, 10.0f, 0.0f, 500.0f);
    config->maximum_slew_deg_per_s = ballPidSanitize(
        ball_pid_ram.maximum_slew_deg_per_s, 60.0f, 1.0f, 720.0f);
}

static float ballPidClamp(float value, float minimum, float maximum)
{
    if (value > maximum)
        return maximum;
    if (value < minimum)
        return minimum;
    return value;
}

static float ballPidAbs(float value)
{
    return value < 0.0f ? -value : value;
}

static float ballPidSanitize(float value,
                             float fallback,
                             float minimum,
                             float maximum)
{
    if ((value != value) || (value < minimum) || (value > maximum))
        return fallback;
    return value;
}
static void oledTask(void *argument)
{
    bool menu_active;
    bool refresh_ok;
    uint32_t last_counter_tick;
    uint32_t now_tick;

    (void)argument;
    last_counter_tick = HAL_GetTick();

    while (1)
    {
        key1ProcessEvents();
        now_tick = HAL_GetTick();
        if ((now_tick - last_counter_tick) >= OLED_COUNTER_INTERVAL_MS)
        {
            oled_default_counter++;
            last_counter_tick = now_tick;
        }
        menu_active = ball_menu_active ||
                      (mode == ROBOT_MODE_REQUIREMENT_3);
        oled.fill(&oled, OLED_COLOR_BLACK);
        if (menu_active)
            oledDrawBallPage();
        else
            oledDrawDefaultPage();

        refresh_ok = oled.refresh(&oled);
        if (!refresh_ok)
        {
            osDelay(5);
            refresh_ok = oled.refresh(&oled);
        }

        if (!refresh_ok)
        {
            oled_refresh_error_count++;
            if (oled.recover(&oled))
                oled_recovery_count++;
        }

        osDelay(50);
    }
}

static void oledDrawDefaultPage(void)
{
    uint32_t display_time_ms;
    bool running;

    taskENTER_CRITICAL();
    running = track_running;
    display_time_ms = running ? track_elapsed_ms : track_total_time_ms;
    taskEXIT_CRITICAL();
    oled.draw_string(&oled, 0, 0, "mode:", OLED_COLOR_WHITE);
    oled.draw_int(&oled, 5, 0, mode, OLED_COLOR_WHITE);
    if (trackModeIsSupported(mode))
    {
        if (mode == ROBOT_MODE_REQUIREMENT_3)
            oled.draw_string(&oled, 0, 2,
                             running ? "R3:RUN" : "R3:STOP",
                             OLED_COLOR_WHITE);
        else if (mode == ROBOT_MODE_REQUIREMENT_4)
            oled.draw_string(&oled, 0, 2,
                             running ? "R4:RUN" : "R4:STOP",
                             OLED_COLOR_WHITE);
        else if (mode == ROBOT_MODE_REQUIREMENT_5)
            oled.draw_string(&oled, 0, 2,
                             running ? "R5:RUN" : "R5:STOP",
                             OLED_COLOR_WHITE);
        else if (mode == ROBOT_MODE_REQUIREMENT_6)
            oled.draw_string(&oled, 0, 2,
                             running ? "R6:RUN" : "R6:STOP",
                             OLED_COLOR_WHITE);
        else
            oled.draw_string(&oled, 0, 2,
                             running ? "R2:RUN" : "R2:STOP",
                             OLED_COLOR_WHITE);
        oled.draw_string(&oled, 0, 4, "time:", OLED_COLOR_WHITE);
        oled.draw_float(&oled, 5, 4,
                        (float)display_time_ms * 0.001f,
                        2, OLED_COLOR_WHITE);
        oled.draw_string(&oled, 11, 4, "s", OLED_COLOR_WHITE);
    }
    oled.draw_string(&oled, 0, 6, "count:", OLED_COLOR_WHITE);
    oled.draw_int(&oled, 6, 6, (int32_t)oled_default_counter,
                  OLED_COLOR_WHITE);
}

static void oledDrawBallPage(void)
{
    float target;
    float real;
    float host_fps;
    float host_period_ms;
    uint32_t display_time_ms;
    bool running;

    taskENTER_CRITICAL();
    target = ball_target;
    real = ball_real;
    host_period_ms = ball_host_period_ema_ms;
    running = track_running;
    display_time_ms = running ? track_elapsed_ms : track_total_time_ms;
    taskEXIT_CRITICAL();
    host_fps = host_period_ms > 0.0f ? 1000.0f / host_period_ms : 0.0f;

    oled.draw_string(&oled, 0, 0, "M3 BALL DEBUG", OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 2, "T:", OLED_COLOR_WHITE);
    oled.draw_int(&oled, 2, 2, (int32_t)(target + 0.5f),
                  OLED_COLOR_WHITE);
    oled.draw_string(&oled, 8, 2, "R:", OLED_COLOR_WHITE);
    oled.draw_int(&oled, 10, 2,
                  real < 0.0f ? (int32_t)(real - 0.5f) :
                                (int32_t)(real + 0.5f),
                  OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 4, "time:", OLED_COLOR_WHITE);
    oled.draw_float(&oled, 5, 4,
                    (float)display_time_ms * 0.001f,
                    2, OLED_COLOR_WHITE);
    oled.draw_string(&oled, 11, 4, "s", OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 6, "fps:", OLED_COLOR_WHITE);
    oled.draw_float(&oled, 4, 6, host_fps, 1, OLED_COLOR_WHITE);
    oled.draw_string(&oled, 10, 6, "q:", OLED_COLOR_WHITE);
    oled.draw_int(&oled, 12, 6,
                  (int32_t)(ball_host_valid_position_count & 0xFFU),
                  OLED_COLOR_WHITE);
}

static void ballHostControlTask(void *argument)
{
    const SlaverSimpleFloatProtocolConfig_t protocol_config = {
        .frame_header = BALL_HOST_FRAME_HEADER,
        .frame_tail = BALL_HOST_FRAME_TAIL,
        .frame_length = BALL_HOST_FRAME_LENGTH,
    };
    const SlaverInitConfig_t receiver_config = {
        .uart_handle = &huart_hc12,
        .dma_config = {
            .dma = DMA,
            .rx_channel = DMA_CH_HC12_RX_CHAN_ID,
            .tx_channel = USART_DMA_CHANNEL_INVALID,
        },
        .uart_irqn = UART_HC12_INST_INT_IRQN,
        .uart_irq_priority = 2U,
        .dma_rx_buffer_size = BALL_HOST_RX_BUFFER_SIZE,
        .header = {BALL_HOST_FRAME_HEADER},
        .header_length = 1U,
        .frame_length_callback = SlaverSimpleFloatFrameLength,
        .frame_validate_callback = SlaverSimpleFloatFrameValidate,
        .frame_callback = SlaverSimpleFloatFrameReceived,
        .protocol_context = &ball_position_protocol,
        .tx_timeout_ms = 100U,
    };
    SlaverSimpleFloatData_t latest;
    uint32_t handled_update_count = 0U;
    uint32_t frame_period_ms;
    int32_t rounded_position;

    (void)argument;
    if (!SlaverSimpleFloatProtocolInit(&ball_position_protocol,
                                       &protocol_config))
    {
        ball_host_init_error = 1U;
        vTaskDelete(NULL);
        return;
    }
    if (!ball_position_receiver.init(&ball_position_receiver,
                                     &receiver_config))
    {
        ball_host_init_error = 2U;
        vTaskDelete(NULL);
        return;
    }

    for (;;)
    {
        ball_position_receiver.process(&ball_position_receiver);
        SlaverSimpleFloatGetLatest(&ball_position_protocol, &latest);
        if (latest.valid &&
            (latest.update_count != handled_update_count))
        {
            handled_update_count = latest.update_count;
            if (ball_host_last_frame_tick != 0U)
            {
                frame_period_ms =
                    latest.last_update_tick -
                    ball_host_last_frame_tick;
                if ((ball_host_period_min_ms == 0U) ||
                    (frame_period_ms < ball_host_period_min_ms))
                {
                    ball_host_period_min_ms = frame_period_ms;
                }
                if (frame_period_ms > ball_host_period_max_ms)
                    ball_host_period_max_ms = frame_period_ms;
                if (ball_host_period_ema_ms == 0.0f)
                    ball_host_period_ema_ms = (float)frame_period_ms;
                else
                    ball_host_period_ema_ms +=
                        0.125f *
                        ((float)frame_period_ms -
                         ball_host_period_ema_ms);
            }
            taskENTER_CRITICAL();
            ball_real = latest.value;
            ball_host_last_frame_tick = latest.last_update_tick;
            taskEXIT_CRITICAL();
            if (latest.value == BALL_HOST_INVALID_POSITION)
            {
                ball_host_invalid_position_count++;
            }
            else if (ballPositionPixelValid(latest.value))
            {
                rounded_position = (int32_t)(latest.value + 0.5f);
                if ((latest.value - (float)rounded_position < 0.001f) &&
                    ((float)rounded_position - latest.value < 0.001f))
                {
                    ball_host_integer_position_count++;
                }
                else
                {
                    ball_host_fractional_position_count++;
                }
                ball_host_valid_position_count++;
            }
            else
            {
                ball_host_rejected_position_count++;
            }
        }

        osDelay(BALL_HOST_TASK_PERIOD_MS);
    }
}

static void trackTask(void *argument)
{
    uint32_t now_tick;
    uint32_t requested_start_tick;
    uint32_t last_control_tick = 0U;
    float control_dt_s;
    (void)argument;
    bool start_requested;
    bool stop_requested;

    for (;;)
    {
        start_requested = false;
        stop_requested = false;
        requested_start_tick = 0U;
        if (trackModeIsSupported(mode))
        {
            taskENTER_CRITICAL();
            if (track_stop_requested)
            {
                stop_requested = true;
                track_stop_requested = false;
                track_start_requested = false;
            }
            if (track_start_requested)
            {
                start_requested = true;
                requested_start_tick = track_start_request_tick;
                track_start_requested = false;
            }
            taskEXIT_CRITICAL();

            if (stop_requested && track_running)
                trackStop(HAL_GetTick());
            else if (start_requested && !track_running)
            {
                now_tick = HAL_GetTick();
                if ((uint32_t)(now_tick - requested_start_tick) >
                    TRACK_START_REQUEST_MAX_AGE_MS)
                {
                    requested_start_tick = now_tick;
                }
                trackStart(requested_start_tick);
            }
        }

        if (track_running)
        {
            now_tick = HAL_GetTick();
            if (!track_timer_stopped)
                track_elapsed_ms = now_tick - track_start_tick;
            if ((last_control_tick == 0U) || (now_tick <= last_control_tick))
            {
                control_dt_s = TRACK_CONTROL_DT_S;
            }
            else
            {
                control_dt_s =
                    (float)(now_tick - last_control_tick) * 0.001f;
                if (control_dt_s < 0.001f)
                    control_dt_s = 0.001f;
                if (control_dt_s > 0.05f)
                    control_dt_s = 0.05f;
            }
            last_control_tick = now_tick;
            if (mode == ROBOT_MODE_REQUIREMENT_3)
                chassis.set_velocity(&chassis, 0.0f, 0.0f);
            else if (mode == ROBOT_MODE_REQUIREMENT_2)
                trackRunRequirement2(now_tick, control_dt_s);
            else if (mode == ROBOT_MODE_REQUIREMENT_4)
                trackRunRequirement4(now_tick, control_dt_s);
            else if ((mode == ROBOT_MODE_REQUIREMENT_5) ||
                     (mode == ROBOT_MODE_REQUIREMENT_6))
                trackRunRequirement5Or6(now_tick, control_dt_s);
            else
                trackStop(now_tick);

            if (track_running &&
                (mode != ROBOT_MODE_REQUIREMENT_3))
            {
                trackDebugCapture(now_tick);
            }
        }
        else
        {
            last_control_tick = 0U;
            ball_chassis_acceleration_mps2 = 0.0f;
            chassis.set_velocity(&chassis, 0.0f, 0.0f);
        }
        osDelay(5);
    }
}

static void trackDebugReset(void)
{
    memset((void *)&track_debug_state, 0, sizeof(track_debug_state));
    track_debug_state.active = 1U;
    track_debug_state.mode = mode;
    track_debug_state.min_command_forward_speed_mps = 1000.0f;
    track_debug_state.min_wheel_speed_scale = 1.0f;
}

static void trackDebugCapture(uint32_t now_tick)
{
    float absolute_error = track.data.normalized_error;
    float absolute_pid_d = track.pid.data.d_out;
    float absolute_base_turn = track.data.turn_speed;
    float absolute_target_turn = chassis.data.target_turn_speed_radps;
    float absolute_command_turn = chassis.data.command_turn_speed_radps;
    float left_speed_error = wheel_left.data.target_linear_speed_mps -
                             wheel_left.data.linear_speed_mps;
    float right_speed_error = wheel_right.data.target_linear_speed_mps -
                              wheel_right.data.linear_speed_mps;
    float turn_limit = track.init_config.max_turn_speed;
    uint32_t black_mask = track.data.black_mask & 0xFFU;
    uint32_t black_count = track.data.black_count;
    uint32_t channel;
    bool line_lost = (black_count == 0U);
    bool outer_sensor_only = (black_mask == 0x01U) ||
                             (black_mask == 0x80U);

    if (absolute_error < 0.0f)
        absolute_error = -absolute_error;
    if (absolute_pid_d < 0.0f)
        absolute_pid_d = -absolute_pid_d;
    if (absolute_base_turn < 0.0f)
        absolute_base_turn = -absolute_base_turn;
    if (absolute_target_turn < 0.0f)
        absolute_target_turn = -absolute_target_turn;
    if (absolute_command_turn < 0.0f)
        absolute_command_turn = -absolute_command_turn;
    if (left_speed_error < 0.0f)
        left_speed_error = -left_speed_error;
    if (right_speed_error < 0.0f)
        right_speed_error = -right_speed_error;

    if (mode == ROBOT_MODE_REQUIREMENT_2)
        turn_limit = TRACK_AGGRESSIVE_MAX_TURN_SPEED;
    else if ((mode == ROBOT_MODE_REQUIREMENT_5) ||
             (mode == ROBOT_MODE_REQUIREMENT_6))
        turn_limit = TRACK_BALL_LAP_MAX_TURN_SPEED;

    track_debug_state.active = 1U;
    track_debug_state.mode = mode;
    track_debug_state.update_count++;
    track_debug_state.elapsed_ms = now_tick - track_start_tick;
    track_debug_state.black_mask = black_mask;
    track_debug_state.black_count = black_count;
    track_debug_state.normalized_error = track.data.normalized_error;
    track_debug_state.last_nonzero_error = track.data.last_nonzero_error;
    track_debug_state.line_lost_time_s = track.data.line_lost_time_s;
    track_debug_state.pid_p_out = track.pid.data.p_out;
    track_debug_state.pid_i_out = track.pid.data.i_out;
    track_debug_state.pid_d_out = track.pid.data.d_out;
    track_debug_state.pid_output = track.pid.data.output;
    track_debug_state.base_turn_speed_radps = track.data.turn_speed;
    track_debug_state.active_turn_limit_radps = turn_limit;
    track_debug_state.target_forward_speed_mps =
        chassis.data.target_forward_speed_mps;
    track_debug_state.target_turn_speed_radps =
        chassis.data.target_turn_speed_radps;
    track_debug_state.command_forward_speed_mps =
        chassis.data.command_forward_speed_mps;
    track_debug_state.command_turn_speed_radps =
        chassis.data.command_turn_speed_radps;
    track_debug_state.measured_forward_speed_mps =
        chassis.data.forward_speed_mps;
    track_debug_state.measured_turn_speed_radps =
        chassis.data.turn_speed_radps;
    track_debug_state.wheel_speed_scale = chassis.data.wheel_speed_scale;
    track_debug_state.left_target_speed_mps =
        wheel_left.data.target_linear_speed_mps;
    track_debug_state.left_measured_speed_mps =
        wheel_left.data.linear_speed_mps;
    track_debug_state.right_target_speed_mps =
        wheel_right.data.target_linear_speed_mps;
    track_debug_state.right_measured_speed_mps =
        wheel_right.data.linear_speed_mps;
    track_debug_state.final_distance_m = chassis.data.distance_m;

    if (absolute_error > track_debug_state.max_abs_error)
    {
        track_debug_state.max_abs_error = absolute_error;
        track_debug_state.max_error_elapsed_ms =
            track_debug_state.elapsed_ms;
        track_debug_state.max_error_black_mask = black_mask;
        track_debug_state.max_error_signed = track.data.normalized_error;
        track_debug_state.max_error_forward_speed_mps =
            chassis.data.command_forward_speed_mps;
        track_debug_state.max_error_turn_speed_radps =
            chassis.data.command_turn_speed_radps;
    }
    if (absolute_pid_d > track_debug_state.max_abs_pid_d_out)
        track_debug_state.max_abs_pid_d_out = absolute_pid_d;
    if (absolute_base_turn >
        track_debug_state.max_abs_base_turn_speed_radps)
    {
        track_debug_state.max_abs_base_turn_speed_radps =
            absolute_base_turn;
    }
    if (absolute_command_turn >
        track_debug_state.max_abs_command_turn_speed_radps)
    {
        track_debug_state.max_abs_command_turn_speed_radps =
            absolute_command_turn;
    }

    if ((chassis.data.command_forward_speed_mps > 0.001f) &&
        (chassis.data.command_forward_speed_mps <
         track_debug_state.min_command_forward_speed_mps))
    {
        track_debug_state.min_command_forward_speed_mps =
            chassis.data.command_forward_speed_mps;
    }
    if (chassis.data.command_forward_speed_mps >
        track_debug_state.max_command_forward_speed_mps)
    {
        track_debug_state.max_command_forward_speed_mps =
            chassis.data.command_forward_speed_mps;
    }
    if (chassis.data.wheel_speed_scale <
        track_debug_state.min_wheel_speed_scale)
    {
        track_debug_state.min_wheel_speed_scale =
            chassis.data.wheel_speed_scale;
    }
    if (left_speed_error > track_debug_state.max_left_speed_error_mps)
        track_debug_state.max_left_speed_error_mps = left_speed_error;
    if (right_speed_error > track_debug_state.max_right_speed_error_mps)
        track_debug_state.max_right_speed_error_mps = right_speed_error;
    if (track.data.line_lost_time_s >
        track_debug_state.max_line_lost_time_s)
    {
        track_debug_state.max_line_lost_time_s =
            track.data.line_lost_time_s;
    }

    if (line_lost)
    {
        if (track_debug_state.line_lost_active == 0U)
            track_debug_state.line_lost_event_count++;
        track_debug_state.line_lost_active = 1U;
        track_debug_state.line_lost_sample_count++;
    }
    else
    {
        track_debug_state.line_lost_active = 0U;
    }
    if (outer_sensor_only)
        track_debug_state.outer_sensor_sample_count++;
    if (absolute_error >= 1.0f)
        track_debug_state.large_error_sample_count++;
    if (absolute_target_turn >= (turn_limit - 0.002f))
        track_debug_state.turn_saturation_sample_count++;
    if (track.data.direction_change_count >
        track_debug_state.max_direction_change_count)
    {
        track_debug_state.max_direction_change_count =
            track.data.direction_change_count;
    }

    for (channel = 0U; channel < 8U; ++channel)
    {
        if ((black_mask & (1UL << channel)) != 0U)
            track_debug_state.sensor_hit_count[channel]++;
    }
    if (black_count <= 8U)
        track_debug_state.black_count_histogram[black_count]++;
}

static void trackDebugStop(uint32_t stop_tick)
{
    track_debug_state.active = 0U;
    track_debug_state.elapsed_ms = stop_tick - track_start_tick;
    track_debug_state.stop_reason = track_stop_reason;
    track_debug_state.final_distance_m = track_stop_distance_m;
}

static bool trackModeIsSupported(uint8_t selected_mode)
{
    return (selected_mode == ROBOT_MODE_REQUIREMENT_2) ||
           (selected_mode == ROBOT_MODE_REQUIREMENT_3) ||
           (selected_mode == ROBOT_MODE_REQUIREMENT_4) ||
           (selected_mode == ROBOT_MODE_REQUIREMENT_5) ||
           (selected_mode == ROBOT_MODE_REQUIREMENT_6);
}

static bool trackModeUsesStableProfile(uint8_t selected_mode)
{
    return (selected_mode == ROBOT_MODE_REQUIREMENT_4) ||
           (selected_mode == ROBOT_MODE_REQUIREMENT_5) ||
           (selected_mode == ROBOT_MODE_REQUIREMENT_6);
}

static bool trackModeUsesBallLapProfile(uint8_t selected_mode)
{
    return (selected_mode == ROBOT_MODE_REQUIREMENT_5) ||
           (selected_mode == ROBOT_MODE_REQUIREMENT_6);
}

static bool trackFinishLineDetected(float distance_m)
{
    uint32_t mask = track.data.black_mask & 0xFFU;
    uint8_t raw_black_count = 0U;
    uint8_t channel;
    bool center_pattern_detected;
    bool support_pattern_detected;
    bool pattern_detected;

    for (channel = 0U; channel < 8U; ++channel)
    {
        if ((mask & (1UL << channel)) != 0U)
            raw_black_count++;
    }

    center_pattern_detected =
        (mask & (mask >> 1U) & (mask >> 2U)) != 0U;
    support_pattern_detected =
        ((mask & TRACK_FINISH_LINE_LEFT_SUPPORT_MASK) ==
         TRACK_FINISH_LINE_LEFT_SUPPORT_MASK) ||
        ((mask & TRACK_FINISH_LINE_RIGHT_SUPPORT_MASK) ==
         TRACK_FINISH_LINE_RIGHT_SUPPORT_MASK);
    pattern_detected =
        center_pattern_detected || support_pattern_detected;
    track_finish_last_mask = (uint8_t)mask;
    track_finish_last_raw_count = raw_black_count;

    if ((track_elapsed_ms >= TRACK_FINISH_LINE_ARM_TIME_MS) &&
        (distance_m >= TRACK_FINISH_ARM_DISTANCE_M) &&
        pattern_detected)
    {
        track_finish_pattern_hit_count++;
        if (center_pattern_detected)
        {
            track_finish_three_channel_hit_count++;
            track_finish_three_channel_seen = true;
            if (track_finish_line_detect_count <=
                (TRACK_FINISH_LINE_MAX_SCORE -
                 TRACK_FINISH_LINE_PRIMARY_SCORE))
            {
                track_finish_line_detect_count +=
                    TRACK_FINISH_LINE_PRIMARY_SCORE;
            }
            else
            {
                track_finish_line_detect_count =
                    TRACK_FINISH_LINE_MAX_SCORE;
            }
        }
        else if (track_finish_line_detect_count <
                 TRACK_FINISH_LINE_MAX_SCORE)
        {
            track_finish_two_channel_hit_count++;
            track_finish_line_detect_count +=
                TRACK_FINISH_LINE_SUPPORT_SCORE;
        }
    }
    else if (track_finish_line_detect_count > 0U)
    {
        track_finish_line_detect_count--;
        if (track_finish_line_detect_count == 0U)
            track_finish_three_channel_seen = false;
    }

    if (((track_finish_line_detect_count >=
          TRACK_FINISH_LINE_CONFIRM_SAMPLES) &&
         track_finish_three_channel_seen) ||
        (track_finish_line_detect_count >=
         TRACK_FINISH_LINE_SUPPORT_ONLY_SCORE))
    {
        track_finish_trigger_count++;
        return true;
    }

    return false;
}

static void trackRunRequirement2(uint32_t now_tick, float dt_s)
{
    ChassisData_t chassis_data;
    float distance_m;
    float remaining_distance_m;
    float forward_speed;
    float absolute_error;
    float turn_speed;
    float turn_speed_limit;
    bool finish_line_detected;

    turn_speed = track.update(&track, dt_s);
    absolute_error = track.data.normalized_error;
    if (absolute_error < 0.0f)
        absolute_error = -absolute_error;
    turn_speed_limit = TRACK_AGGRESSIVE_MAX_TURN_SPEED;
    if ((track.data.black_count == 0U) ||
        (track.data.black_mask == 0x01U) ||
        (track.data.black_mask == 0x80U))
    {
        turn_speed_limit = TRACK_RECOVERY_MAX_TURN_SPEED;
    }
    else if (absolute_error >= 2.0f)
    {
        turn_speed *= TRACK_LARGE_ERROR_TURN_GAIN;
        turn_speed_limit = TRACK_RECOVERY_MAX_TURN_SPEED;
    }
    if (turn_speed > turn_speed_limit)
        turn_speed = turn_speed_limit;
    else if (turn_speed < -turn_speed_limit)
        turn_speed = -turn_speed_limit;
    chassis.get_data(&chassis, &chassis_data);
    distance_m = (chassis_data.distance_m >= 0.0f) ?
                 chassis_data.distance_m : -chassis_data.distance_m;
    remaining_distance_m = TRACK_LAP_DISTANCE_M - distance_m;

    if (track_finish_reverse_active)
    {
        float reverse_distance_m =
            track_finish_reverse_start_distance_m -
            chassis_data.distance_m;

        if ((reverse_distance_m >= TRACK_FINISH_REVERSE_DISTANCE_M) ||
            ((now_tick - track_finish_reverse_start_tick) >=
             TRACK_FINISH_REVERSE_MAX_TIME_MS))
        {
            trackStop(now_tick);
        }
        else
        {
            chassis.set_velocity(
                &chassis,
                -TRACK_FINISH_REVERSE_SPEED_MPS,
                0.0f);
        }
        return;
    }

    finish_line_detected = trackFinishLineDetected(distance_m);

    if ((!track_finish_align_active) && finish_line_detected)
    {
        track_stop_reason = 1U;
        track_finish_trigger_distance_m = distance_m;
        track_finish_align_active = true;
        track_finish_align_settle_count = 0U;
        track_finish_align_start_turn_angle_rad =
            chassis_data.turn_angle_rad;
        track_finish_align_error_rad = 0.0f;
        track_finish_align_start_tick = now_tick;
        track.pid.reset(&track.pid);
    }

    if (track_finish_align_active)
    {
        float rotation_from_stop_rad;

        track_finish_align_error_rad = -trackWrapAngleRad(
            chassis_data.turn_angle_rad);
        rotation_from_stop_rad = trackWrapAngleRad(
            chassis_data.turn_angle_rad -
            track_finish_align_start_turn_angle_rad);
        absolute_error = track_finish_align_error_rad;
        if (absolute_error < 0.0f)
            absolute_error = -absolute_error;
        if (absolute_error <= TRACK_FINISH_ALIGN_HEADING_TOLERANCE_RAD)
        {
            if (track_finish_align_settle_count <
                TRACK_FINISH_ALIGN_CONFIRM_SAMPLES)
            {
                track_finish_align_settle_count++;
            }
        }
        else
        {
            track_finish_align_settle_count = 0U;
        }

        if ((track_finish_align_settle_count >=
             TRACK_FINISH_ALIGN_CONFIRM_SAMPLES) ||
            ((rotation_from_stop_rad >= TRACK_FINISH_ALIGN_MAX_ROTATION_RAD) ||
             (rotation_from_stop_rad <= -TRACK_FINISH_ALIGN_MAX_ROTATION_RAD)) ||
            ((now_tick - track_finish_align_start_tick) >=
             TRACK_FINISH_ALIGN_MAX_TIME_MS))
        {
            chassis.set_velocity(&chassis, 0.0f, 0.0f);
            track_finish_align_active = false;
            track_finish_reverse_active = true;
            track_finish_reverse_start_distance_m =
                chassis_data.distance_m;
            track_finish_reverse_start_tick = now_tick;
        }
        else
        {
            turn_speed = TRACK_FINISH_ALIGN_HEADING_KP *
                         track_finish_align_error_rad;
            if (turn_speed > TRACK_FINISH_ALIGN_MAX_TURN_SPEED)
                turn_speed = TRACK_FINISH_ALIGN_MAX_TURN_SPEED;
            else if (turn_speed < -TRACK_FINISH_ALIGN_MAX_TURN_SPEED)
                turn_speed = -TRACK_FINISH_ALIGN_MAX_TURN_SPEED;
            if ((turn_speed > -TRACK_FINISH_ALIGN_MIN_TURN_SPEED) &&
                (turn_speed < TRACK_FINISH_ALIGN_MIN_TURN_SPEED))
            {
                turn_speed = track_finish_align_error_rad < 0.0f ?
                             -TRACK_FINISH_ALIGN_MIN_TURN_SPEED :
                             TRACK_FINISH_ALIGN_MIN_TURN_SPEED;
            }
            chassis.set_velocity(&chassis, 0.0f, turn_speed);
        }
    }
    else if (remaining_distance_m <= TRACK_POSITION_TOLERANCE_M)
    {
        track_stop_reason = 2U;
        trackStop(now_tick);
    }
    else if (track_elapsed_ms >= TRACK_MAX_RUN_TIME_MS)
    {
        track_stop_reason = 3U;
        trackStop(now_tick);
    }
    else
    {
        forward_speed = track.init_config.base_speed;
        forward_speed = trackLimitForwardSpeed(forward_speed);
        if ((distance_m >= TRACK_FINISH_ARM_DISTANCE_M) &&
            (forward_speed > TRACK_MODE2_FINISH_APPROACH_SPEED_MPS))
        {
            forward_speed = TRACK_MODE2_FINISH_APPROACH_SPEED_MPS;
        }
        forward_speed = trackSmoothForwardSpeed(forward_speed, dt_s);
        chassis.set_velocity(&chassis, forward_speed, -turn_speed);
    }
}

static void trackRunRequirement4(uint32_t now_tick, float dt_s)
{
    ChassisData_t chassis_data;
    float absolute_error;
    float distance_m;
    float turn_speed;
    bool curve_signal;
    bool outer_sensor_only;

    turn_speed = track.update(&track, dt_s);
    chassis.get_data(&chassis, &chassis_data);
    distance_m = (chassis_data.distance_m >= 0.0f) ?
                 chassis_data.distance_m : -chassis_data.distance_m;
    absolute_error = track.data.normalized_error;
    if (absolute_error < 0.0f)
        absolute_error = -absolute_error;
    outer_sensor_only =
        (track.data.black_mask == 0x01U) ||
        (track.data.black_mask == 0x80U);
    curve_signal = (distance_m >= REQUIREMENT4_CURVE_ARM_DISTANCE_M) &&
                   (outer_sensor_only ||
                    (absolute_error >= REQUIREMENT4_CURVE_ERROR_THRESHOLD));

    if (curve_signal)
    {
        if (requirement4_curve_detect_count <
            REQUIREMENT4_CURVE_CONFIRM_SAMPLES)
        {
            requirement4_curve_detect_count++;
        }
    }
    else
    {
        requirement4_curve_detect_count = 0U;
    }

    if ((distance_m >= REQUIREMENT4_AB_DISTANCE_M) ||
        (requirement4_curve_detect_count >=
         REQUIREMENT4_CURVE_CONFIRM_SAMPLES) ||
        (track_elapsed_ms >= REQUIREMENT4_MAX_RUN_TIME_MS))
    {
        trackStop(now_tick);
    }
    else
    {
        chassis.set_velocity(
            &chassis,
            trackSmoothForwardSpeed(
                trackLimitForwardSpeed(REQUIREMENT4_FORWARD_SPEED_MPS),
                dt_s),
            -turn_speed);
    }
}

/* Requirement 5/6 currently reuse the one-lap route; their route actions can
 * be added independently without changing the stable chassis profile. */
static void trackRunRequirement5Or6(uint32_t now_tick, float dt_s)
{
    ChassisData_t chassis_data;
    float distance_m;
    float forward_speed;
    float turn_speed;

    turn_speed = track.update(&track, dt_s);
    turn_speed *= TRACK_BALL_LAP_TURN_GAIN;
    if (turn_speed > TRACK_BALL_LAP_MAX_TURN_SPEED)
        turn_speed = TRACK_BALL_LAP_MAX_TURN_SPEED;
    else if (turn_speed < -TRACK_BALL_LAP_MAX_TURN_SPEED)
        turn_speed = -TRACK_BALL_LAP_MAX_TURN_SPEED;

    chassis.get_data(&chassis, &chassis_data);
    distance_m = (chassis_data.distance_m >= 0.0f) ?
                 chassis_data.distance_m : -chassis_data.distance_m;
    (void)trackFinishLineDetected(distance_m);

    if (track_elapsed_ms >= TRACK_STABLE_MAX_RUN_TIME_MS)
    {
        track_stop_reason = 3U;
        trackStop(now_tick);
    }
    else
    {
        forward_speed = TRACK_BALL_LAP_BASE_SPEED_MPS;
        forward_speed = trackLimitForwardSpeed(forward_speed);
        forward_speed = trackSmoothForwardSpeed(forward_speed, dt_s);
        chassis.set_velocity(&chassis, forward_speed, -turn_speed);
    }
}

static void trackStart(uint32_t start_tick)
{
    float latest_ball_position;
    float start_position_error;
    uint32_t latest_ball_tick;
    uint32_t now_tick;

    chassis.set_velocity(&chassis, 0.0f, 0.0f);
    if (mode == ROBOT_MODE_REQUIREMENT_3)
    {
        taskENTER_CRITICAL();
        latest_ball_position = ball_real;
        latest_ball_tick = ball_host_last_frame_tick;
        taskEXIT_CRITICAL();
        now_tick = HAL_GetTick();
        start_position_error = latest_ball_position - ball_target;
        if (start_position_error < 0.0f)
            start_position_error = -start_position_error;
        if ((latest_ball_tick == 0U) ||
            ((now_tick - latest_ball_tick) >
             BALL_MODE3_START_MAX_AGE_MS) ||
            ((latest_ball_position != BALL_HOST_INVALID_POSITION) &&
             ((!ballPositionPixelValid(latest_ball_position)) ||
              (start_position_error >
               BALL_MODE3_START_TOLERANCE_PX))))
        {
            track_elapsed_ms = 0U;
            track_total_time_ms = 0U;
            track_timer_stopped = true;
            ball_test_error = 3U;
            led_red.off(&led_red);
            return;
        }
        if (ball_test_error == 3U)
            ball_test_error = 0U;
    }
    chassis.reset_odometry(&chassis);
    track.pid.reset(&track.pid);
    taskENTER_CRITICAL();
    track_start_tick = start_tick;
    track_elapsed_ms = 0U;
    track_total_time_ms = 0U;
    track_timer_stopped = false;
    taskEXIT_CRITICAL();
    track_slow_until_tick = start_tick;
    track_forward_speed_command = 0.0f;
    ball_chassis_acceleration_mps2 = 0.0f;
    requirement4_curve_detect_count = 0U;
    track_finish_line_detect_count = 0U;
    track_finish_last_mask = 0U;
    track_finish_last_raw_count = 0U;
    track_finish_pattern_hit_count = 0U;
    track_finish_trigger_count = 0U;
    track_finish_three_channel_hit_count = 0U;
    track_finish_two_channel_hit_count = 0U;
    track_finish_trigger_distance_m = 0.0f;
    track_stop_distance_m = 0.0f;
    track_finish_three_channel_seen = false;
    track_stop_reason = 0U;
    track_finish_align_active = false;
    track_finish_align_settle_count = 0U;
    track_finish_align_start_turn_angle_rad = 0.0f;
    track_finish_align_error_rad = 0.0f;
    track_finish_align_start_tick = 0U;
    track_finish_reverse_active = false;
    track_finish_reverse_start_distance_m = 0.0f;
    track_finish_reverse_start_tick = 0U;
    trackDebugReset();
    track_running = true;
    led_red.on(&led_red);
}

static void trackStop(uint32_t stop_tick)
{
    ChassisData_t chassis_data;

    chassis.set_velocity(&chassis, 0.0f, 0.0f);
    chassis.get_data(&chassis, &chassis_data);
    track_stop_distance_m = chassis_data.distance_m;
    track.pid.reset(&track.pid);
    track_forward_speed_command = 0.0f;
    ball_chassis_acceleration_mps2 = 0.0f;
    taskENTER_CRITICAL();
    if (!track_timer_stopped)
        track_elapsed_ms = stop_tick - track_start_tick;
    track_total_time_ms = track_elapsed_ms;
    track_timer_stopped = true;
    taskEXIT_CRITICAL();
    track_running = false;
    trackDebugStop(stop_tick);
    led_red.off(&led_red);
}

static float trackLimitForwardSpeed(float requested_speed_mps)
{
    float absolute_error = track.data.normalized_error;
    float speed_scale = 1.0f;
    uint32_t now_tick = HAL_GetTick();
    bool stable_profile = trackModeUsesStableProfile(mode);
    bool ball_lap_profile = trackModeUsesBallLapProfile(mode);
    bool outer_sensor_only =
        (track.data.black_mask == 0x01U) ||
        (track.data.black_mask == 0x80U);

    if (absolute_error < 0.0f)
        absolute_error = -absolute_error;

    if (outer_sensor_only || (absolute_error >= 1.0f))
        track_slow_until_tick = now_tick + TRACK_CORNER_EXIT_HOLD_MS;

    if (ball_lap_profile)
    {
        if (track.data.black_count == 0U)
            speed_scale = 0.07f;
        else if (outer_sensor_only || (absolute_error >= 2.0f))
            speed_scale = 0.90f;
        else if (absolute_error >= 1.0f)
            speed_scale = 0.95f;
    }
    else if (stable_profile)
    {
        if (track.data.black_count == 0U)
            speed_scale = 0.07f;
        else if (outer_sensor_only)
            speed_scale = 0.36f;
        else if (absolute_error >= 2.0f)
            speed_scale = 0.46f;
        else if (absolute_error >= 1.0f)
            speed_scale = 0.65f;
    }
    else
    {
        if (track.data.black_count == 0U)
        {
            speed_scale =
                (track.data.line_lost_time_s <
                 TRACK_MODE2_TRANSIENT_LOSS_TIME_S) ?
                TRACK_MODE2_TRANSIENT_LOSS_SPEED_SCALE :
                TRACK_MODE2_LOST_LINE_SPEED_SCALE;
        }
        else if (outer_sensor_only)
            speed_scale = TRACK_MODE2_OUTER_SENSOR_SPEED_SCALE;
        else if (absolute_error >= 3.0f)
            speed_scale = TRACK_MODE2_LARGE_ERROR_SPEED_SCALE;
        else if (absolute_error >= 2.0f)
            speed_scale = TRACK_MODE2_LARGE_ERROR_SPEED_SCALE;
        else if (absolute_error >= 1.0f)
            speed_scale = TRACK_MODE2_LARGE_ERROR_SPEED_SCALE;
    }

    if (((int32_t)(track_slow_until_tick - now_tick) > 0) &&
        (speed_scale > (ball_lap_profile ? 0.90f :
                        (stable_profile ? 0.46f :
                         TRACK_MODE2_CORNER_HOLD_SPEED_SCALE))))
    {
        speed_scale = ball_lap_profile ? 0.90f :
                      (stable_profile ? 0.46f :
                       TRACK_MODE2_CORNER_HOLD_SPEED_SCALE);
    }

    return requested_speed_mps * speed_scale;
}

static float trackSmoothForwardSpeed(float target_speed_mps, float dt_s)
{
    float maximum_step;
    float previous_speed_mps = track_forward_speed_command;
    bool stable_profile = trackModeUsesStableProfile(mode);
    bool ball_lap_profile = trackModeUsesBallLapProfile(mode);

    if (dt_s <= 0.0f)
        dt_s = TRACK_CONTROL_DT_S;

    if (stable_profile &&
        (track_elapsed_ms < TRACK_STABLE_SOFT_START_MS))
    {
        float progress =
            (float)track_elapsed_ms /
            (float)TRACK_STABLE_SOFT_START_MS;
        float smooth_progress =
            progress * progress * (3.0f - (2.0f * progress));

        target_speed_mps *= smooth_progress;
    }

    if (target_speed_mps > track_forward_speed_command)
    {
        maximum_step =
            (ball_lap_profile ? TRACK_BALL_LAP_FORWARD_ACCEL_MPS2 :
             (stable_profile ? TRACK_STABLE_FORWARD_ACCEL_MPS2 :
              TRACK_FORWARD_ACCEL_MPS2)) * dt_s;
        if ((target_speed_mps - track_forward_speed_command) > maximum_step)
            target_speed_mps = track_forward_speed_command + maximum_step;
    }
    else
    {
        maximum_step =
            (ball_lap_profile ? TRACK_BALL_LAP_FORWARD_DECEL_MPS2 :
             (stable_profile ? TRACK_STABLE_FORWARD_DECEL_MPS2 :
              TRACK_FORWARD_DECEL_MPS2)) * dt_s;
        if ((track_forward_speed_command - target_speed_mps) > maximum_step)
            target_speed_mps = track_forward_speed_command - maximum_step;
    }

    track_forward_speed_command = target_speed_mps;
    /* 使用限加减速后的速度指令计算前馈，匀速时自动回零。 */
    if (stable_profile)
    {
        ball_chassis_acceleration_mps2 =
            (track_forward_speed_command - previous_speed_mps) / dt_s;
    }
    else
    {
        ball_chassis_acceleration_mps2 = 0.0f;
    }
    return track_forward_speed_command;
}

static float trackWrapAngleRad(float angle_rad)
{
    while (angle_rad > 3.14159265f)
        angle_rad -= 6.28318531f;
    while (angle_rad < -3.14159265f)
        angle_rad += 6.28318531f;
    return angle_rad;
}

static bool ballPositionPixelValid(float position_px)
{
    return (position_px == position_px) &&
           (position_px >= BALL_POSITION_MIN_PX) &&
           (position_px <= BALL_POSITION_MAX_PX);
}

void UART_HC12_INST_IRQHandler(void)
{
    USARTIRQHandler(&huart_hc12);
}

void keyEventCallback(Key_t *key, KeyEvent_t event, void *context)
{
    (void)context;

    if(key == &key2)
    {
        if (event == KEY_EVENT_PRESS)
        {
            if (trackModeIsSupported(mode))
            {
                if (track_running || track_start_requested)
                {
                    track_stop_requested = true;
                }
                else
                {
                    track_start_request_tick = HAL_GetTickFromISR();
                    track_start_requested = true;
                }
            }
            else
            {
                track_start_requested = false;
                track_running = false;
                led_red.off(&led_red);
            }
        }
    }
}

static void keyA07HandleInterrupt(void)
{
    uint32_t now_tick = HAL_GetTickFromISR();

    if ((now_tick - key_a07_last_toggle_tick) < KEY_A07_DEBOUNCE_MS)
        return;

    key_a07_last_toggle_tick = now_tick;
    if (ball_pid_ram.enabled != 0U)
    {
        ball_pid_ram.enabled = 0U;
        led_green.off(&led_green);
    }
    else
    {
        ball_pid_ram.enabled = 1U;
        led_green.on(&led_green);
    }
}

static void key1ProcessEvents(void)
{
    uint32_t now_tick = HAL_GetTick();
    bool sampled_pressed;
    bool toggle_menu = false;
    bool short_press = false;

    sampled_pressed = (key1.read(&key1) == KEY_STATE_PRESSED);
    if (sampled_pressed != key1_sample_pressed)
    {
        key1_sample_pressed = sampled_pressed;
        key1_sample_change_tick = now_tick;
    }

    if ((key1_sample_pressed != key1_stable_pressed) &&
        ((now_tick - key1_sample_change_tick) >= key1.init_config.debounce_ms))
    {
        key1_stable_pressed = key1_sample_pressed;
        if (key1_stable_pressed)
        {
            key1_press_tick = now_tick;
            key1_long_press_handled = false;
        }
        else if (!key1_long_press_handled)
        {
            key1_long_press_handled = true;
            short_press = true;
        }
    }

    if (key1_stable_pressed && (!key1_long_press_handled) &&
        ((now_tick - key1_press_tick) >= 2000U))
    {
        key1_long_press_handled = true;
        toggle_menu = true;
    }

    if (toggle_menu)
    {
        ball_menu_active = !ball_menu_active;
    }
    else if (short_press)
    {
        if (ball_menu_active)
        {
            ball_target = ball_real;
        }
        else if (!track_running && !track_start_requested)
        {
            if (mode == ROBOT_MODE_REQUIREMENT_2)
                mode = ROBOT_MODE_REQUIREMENT_3;
            else if (mode == ROBOT_MODE_REQUIREMENT_3)
                mode = ROBOT_MODE_REQUIREMENT_4;
            else if (mode == ROBOT_MODE_REQUIREMENT_4)
                mode = ROBOT_MODE_REQUIREMENT_5;
            else if (mode == ROBOT_MODE_REQUIREMENT_5)
                mode = ROBOT_MODE_REQUIREMENT_6;
            else
                mode = ROBOT_MODE_REQUIREMENT_2;
            if (!trackModeIsSupported(mode))
            {
                track_start_requested = false;
                if (track_running)
                    trackStop(now_tick);
                else
                    led_red.off(&led_red);
            }
        }
    }
}

static void ledInit(void){
    LedInitConfig_t led_green_config = {
        .GPIOx        = GPIOB,
        .GPIO_Pin     = GPIO_PIN_11,
        .active_state = GPIO_PIN_SET,   
        .init_state   = LED_STATE_OFF,
        .id           = NULL            
    };
    LedInitConfig_t led_red_config = {
        .GPIOx        = GPIOB,
        .GPIO_Pin     = GPIO_PIN_10,
        .active_state = GPIO_PIN_SET,   
        .init_state   = LED_STATE_OFF,
        .id           = NULL            
    };
    LedInitConfig_t led_blue_config = {
        .GPIOx        = GPIOB,
        .GPIO_Pin     = GPIO_PIN_0,
        .active_state = GPIO_PIN_SET,   
        .init_state   = LED_STATE_OFF,
        .id           = NULL            
    };

    led_green.init(&led_green, &led_green_config);
    led_red.init(&led_red, &led_red_config);
    led_blue.init(&led_blue, &led_blue_config);
}

void oledInit(void){
    const OledInitConfig_t oled_config = {
        .i2c_handle = NULL,
        .iic_bus_mode = IIC_BUS_SOFTWARE,
        .soft_iic = {
            .scl = {
                .GPIOx      = GPIOA,
                .GPIO_Pin   = GPIO_PIN_15,
                .GPIO_IOMUX = GPIO_OLED_OLED_SCL_A15_IOMUX,
            },
            .sda = {
                .GPIOx      = GPIOA,
                .GPIO_Pin   = GPIO_PIN_17,
                .GPIO_IOMUX = GPIO_OLED_OLED_SDA_A17_IOMUX,
            },
            .delay_us = 5U,
        },
    };
    
    
    oled.init(&oled, &oled_config);
}

static void keyInit(void){
    const KeyInitConfig_t key1_config = {
        .GPIOx          = GPIOA,
        .GPIO_Pin       = GPIO_PIN_18,
        .active_state   = GPIO_PIN_RESET,
        .exti_mode      = GPIO_EXTI_MODE_RISING_FALLING,
        .debounce_ms    = 50U,
        .event_callback = keyEventCallback,
        .event_context  = NULL,
    };
    const KeyInitConfig_t key2_config = {
        .GPIOx          = GPIOB,
        .GPIO_Pin       = GPIO_PIN_14,
        .active_state   = GPIO_PIN_RESET,
        .exti_mode      = GPIO_EXTI_MODE_RISING_FALLING,
        .debounce_ms    = 50U,
        .event_callback = keyEventCallback,
        .event_context  = NULL,
    };
    key1.init(&key1, &key1_config);
    key2.init(&key2, &key2_config);
    key1_sample_pressed = (key1.read(&key1) == KEY_STATE_PRESSED);
    key1_stable_pressed = key1_sample_pressed;
    key1_sample_change_tick = HAL_GetTick();
    key1_press_tick = key1_sample_change_tick;
    key_a07_last_toggle_tick = HAL_GetTick() - KEY_A07_DEBOUNCE_MS;

    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_SetPriority(GPIO_MULTIPLE_GPIOA_INT_IRQN, 2U);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_SetPriority(GPIO_MULTIPLE_GPIOB_INT_IRQN, 2U);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case GPIO_MULTIPLE_GPIOA_INT_IIDX:
            gpioInterruptDispatch(GPIOA);
            break;

        case GPIO_MULTIPLE_GPIOB_INT_IIDX:
            gpioInterruptDispatch(GPIOB);
            break;

        default:
            break;
    }
}

static void gpioInterruptDispatch(GPIO_TypeDef *GPIOx)
{
    DL_GPIO_IIDX pending;

    while ((pending = DL_GPIO_getPendingInterrupt(GPIOx)) != DL_GPIO_IIDX_NO_INTR)
    {
        if (GPIOx == GPIOA)
        {
            switch (pending)
            {
                case GPIO_KEY_KEY_A07_IIDX:
                    keyA07HandleInterrupt();
                    break;
                case GPIO_KEY_KEY_A18_IIDX:
                    GPIOIRQHandler(GPIOx, GPIO_KEY_KEY_A18_PIN);
                    break;
                case GPIO_ENCODER_ENCODER_RIGHT_A_A08_IIDX:
                    GPIOIRQHandler(GPIOx, GPIO_ENCODER_ENCODER_RIGHT_A_A08_PIN);
                    break;
                case GPIO_ENCODER_ENCODER_RIGHT_B_A09_IIDX:
                    GPIOIRQHandler(GPIOx, GPIO_ENCODER_ENCODER_RIGHT_B_A09_PIN);
                    break;
                default:
                    break;
            }
        }
        else if (GPIOx == GPIOB)
        {
            switch (pending)
            {
                case GPIO_KEY_KEY_B14_IIDX:
                    GPIOIRQHandler(GPIOx, GPIO_KEY_KEY_B14_PIN);
                    break;
                case GPIO_ENCODER_ENCODER_LEFT_A_B06_IIDX:
                    GPIOIRQHandler(GPIOx, GPIO_ENCODER_ENCODER_LEFT_A_B06_PIN);
                    break;
                case GPIO_ENCODER_ENCODER_LEFT_B_B07_IIDX:
                    GPIOIRQHandler(GPIOx, GPIO_ENCODER_ENCODER_LEFT_B_B07_PIN);
                    break;
                default:
                    break;
            }
        }
    }
}

void grayInit(void){
    const GrayChannelConfig_t gray_channels[] = {
        { .GPIOx = GPIOA, .GPIO_Pin = GPIO_PIN_2 },   // GRAY_A02  第0路
        { .GPIOx = GPIOB, .GPIO_Pin = GPIO_PIN_19 },  // GRAY_B19  第1路
        { .GPIOx = GPIOB, .GPIO_Pin = GPIO_PIN_17 },  // GRAY_B17  第2路
        { .GPIOx = GPIOA, .GPIO_Pin = GPIO_PIN_16 },  // GRAY_A16  第3路
        { .GPIOx = GPIOA, .GPIO_Pin = GPIO_PIN_14 },  // GRAY_A14  第4路
        { .GPIOx = GPIOB, .GPIO_Pin = GPIO_PIN_20 },  // GRAY_B20  第5路
        { .GPIOx = GPIOB, .GPIO_Pin = GPIO_PIN_25 },  // GRAY_B25  第6路
        { .GPIOx = GPIOA, .GPIO_Pin = GPIO_PIN_25 },  // GRAY_A25  第7路
    };

    const GrayInitConfig_t gray_config = {
        .channels      = gray_channels,
        .channel_count = sizeof(gray_channels) / sizeof(gray_channels[0]),
        .black_state   = GPIO_PIN_RESET,
    };

    gray.init(&gray, &gray_config);
}

static void trackInit(void){
    const TrackInitConfig_t track_config = {
        .gray = &gray,
        .channel_count = 8U,
        .weights = { -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f },
        .base_speed = 0.58f,
        .max_turn_speed = 0.85f,
        .pid_config = {
            .kp = 0.235f,
            .ki = 0.0f,
            .kd = 0.0015f,
            .enable_output_limit = true,
            .output_min = -0.85f,
            .output_max = 0.85f,
            .enable_integral_limit = true,
            .integral_min = -1.0f,
            .integral_max = 1.0f,
            .deadband = 0.0f,
            .derivative_filter_tau_s = 0.03f,
            .derivative_on_measurement = false,
            .reset_integral_on_deadband = true,
        },
    };
    track.init(&track, &track_config);
}

static void encoderInit(void){
    const EncoderInitConfig_t encoder_left_config = {
        .mode = ENCODER_MODE_SOFTWARE_GPIO,
        .phase_a = {
            .GPIOx = GPIO_ENCODER_ENCODER_LEFT_A_B06_PORT,
            .GPIO_Pin = GPIO_ENCODER_ENCODER_LEFT_A_B06_PIN,
        },
        .phase_b = {
            .GPIOx = GPIO_ENCODER_ENCODER_LEFT_B_B07_PORT,
            .GPIO_Pin = GPIO_ENCODER_ENCODER_LEFT_B_B07_PIN,
        },
        .reversed = false,
        .counts_per_rev = 1560.0f,
        .auto_start = true,
        .speed_window_samples = 4U,
    };
    const EncoderInitConfig_t encoder_right_config = {
        .mode = ENCODER_MODE_SOFTWARE_GPIO,
        .phase_a = {
            .GPIOx = GPIO_ENCODER_ENCODER_RIGHT_A_A08_PORT,
            .GPIO_Pin = GPIO_ENCODER_ENCODER_RIGHT_A_A08_PIN,
        },
        .phase_b = {
            .GPIOx = GPIO_ENCODER_ENCODER_RIGHT_B_A09_PORT,
            .GPIO_Pin = GPIO_ENCODER_ENCODER_RIGHT_B_A09_PIN,
        },
        .reversed = true,
        .counts_per_rev = 1560.0f,
        .auto_start = true,
        .speed_window_samples = 4U,
    };
    configASSERT(encoder_left.init(&encoder_left, &encoder_left_config));
    configASSERT(encoder_right.init(&encoder_right, &encoder_right_config));
}

static void motorInit(void){
    const MotorInitConfig_t motor_left_config = {
        .type            = MOTOR_TYPE_REDUCTION,
        .use_reverse_pwm = false,
        .pwm = {
            .htim      = &htim_motor_pwm,
            .channel   = TIM_CHANNEL_1,
            .period    = 50.0e-6f,
            .init_duty = 0.0f,
        },
        .use_tb6612 = true,
        .tb6612 = {
            .in1 = {
                .GPIOx    = GPIO_MOTOR_DIR_MOTOR_AIN1_A12_PORT,
                .GPIO_Pin = GPIO_MOTOR_DIR_MOTOR_AIN1_A12_PIN,
            },
            .in2 = {
                .GPIOx    = GPIO_MOTOR_DIR_MOTOR_AIN2_A13_PORT,
                .GPIO_Pin = GPIO_MOTOR_DIR_MOTOR_AIN2_A13_PIN,
            },
            .use_standby   = false,
            .brake_on_stop = true,
            .reversed      = true,
        },
        .use_encoder = true,
        .encoder     = &encoder_left,
        .min_duty    = 0.10f,
        .max_duty    = 1.0f,
        .init_output = 0.0f,
    };
    const MotorInitConfig_t motor_right_config = {
        .type            = MOTOR_TYPE_REDUCTION,
        .use_reverse_pwm = false,
        .pwm = {
            .htim      = &htim_motor_pwm,
            .channel   = TIM_CHANNEL_2,
            .period    = 50.0e-6f,
            .init_duty = 0.0f,
        },
        .use_tb6612 = true,
        .tb6612 = {
            .in1 = {
                .GPIOx    = GPIO_MOTOR_DIR_MOTOR_BIN1_B08_PORT,
                .GPIO_Pin = GPIO_MOTOR_DIR_MOTOR_BIN1_B08_PIN,
            },
            .in2 = {
                .GPIOx    = GPIO_MOTOR_DIR_MOTOR_BIN2_B09_PORT,
                .GPIO_Pin = GPIO_MOTOR_DIR_MOTOR_BIN2_B09_PIN,
            },
            .use_standby   = false,
            .brake_on_stop = true,
            .reversed      = true,
        },
        .use_encoder = true,
        .encoder     = &encoder_right,
        .min_duty    = 0.10f,
        .max_duty    = 1.0f,
        .init_output = 0.0f,
    };
    const MotorInitConfig_t motor_ball_servo_config = {
        .type = MOTOR_TYPE_SERVO,
        .pwm = {
            .htim = &htim_ball_servo_pwm,
            .channel = TIM_CHANNEL_1,
            .period = 0.020f,
            .init_duty = 0.0f,
        },
        .use_encoder = false,
        .min_duty = 0.0f,
        .max_duty = 1.0f,
        .init_output = BALL_SERVO_CENTER_ANGLE_DEG,
        .servo_min_angle = 0.0f,
        .servo_max_angle = 270.0f,
        .servo_min_pulse_us = 500.0f,
        .servo_max_pulse_us = 2500.0f,
    };
    configASSERT(motor_left.init(&motor_left, &motor_left_config));
    configASSERT(motor_right.init(&motor_right, &motor_right_config));
    configASSERT(motor_ball_servo.init(&motor_ball_servo,
                                       &motor_ball_servo_config));
}

static void pidInit(void)
{
    const PidInitConfig_t wheel_left_pid_config = {
        .kp = 2.8f,
        .ki = 0.5f,
        .kd = 0.0f,
        .enable_output_limit = true,
        .output_min = -1.0f,
        .output_max = 1.0f,
        .enable_integral_limit = true,
        .integral_min = -0.5f,
        .integral_max = 0.5f,
        .deadband     = 0.003f,
        .derivative_filter_tau_s = 0.0f,
        .derivative_on_measurement  = true,
        .reset_integral_on_deadband = false,
    };
    const PidInitConfig_t wheel_right_pid_config = {
        .kp = 2.8f,
        .ki = 0.5f,
        .kd = 0.0f,
        .enable_output_limit = true,
        .output_min = -1.0f,
        .output_max = 1.0f,
        .enable_integral_limit = true,
        .integral_min = -0.5f,
        .integral_max = 0.5f,
        .deadband     = 0.003f,
        .derivative_filter_tau_s = 0.0f,
        .derivative_on_measurement  = true,
        .reset_integral_on_deadband = false,
    };
    configASSERT(wheel_left_pid.init(&wheel_left_pid, &wheel_left_pid_config));
    configASSERT(wheel_right_pid.init(&wheel_right_pid, &wheel_right_pid_config));
}

static void wheelInit(void)
{
    const WheelInitConfig_t wheel_left_config = {
        .motor = &motor_left,
        .speed_pid = &wheel_left_pid,
        .use_speed_pid = true,
        .radius_m = 0.0325f,
        .encoder_to_wheel_ratio = 1.0f,
        .max_linear_speed_mps   = 1.2f,
        .output_min = -1.0f,
        .output_max = 1.0f,
        .speed_pid_no_reverse = true,
        .reversed = false,
        .auto_start = false,
    };
    const WheelInitConfig_t wheel_right_config = {
        .motor = &motor_right,
        .speed_pid = &wheel_right_pid,
        .use_speed_pid = true,
        .radius_m = 0.0325f,
        .encoder_to_wheel_ratio = 1.0f,
        .max_linear_speed_mps   = 1.2f,
        .output_min = -1.0f,
        .output_max = 1.0f,
        .speed_pid_no_reverse = true,
        .reversed = false,
        .auto_start = false,
    };
    configASSERT(wheel_left.init(&wheel_left, &wheel_left_config));
    configASSERT(wheel_right.init(&wheel_right, &wheel_right_config));
}

static void chassisInit(void)
{
    static Wheel_t *const chassis_wheels[2] = {
        &wheel_left,
        &wheel_right
    };
    static const ChassisWheelKinematics_t chassis_wheel_kinematics[2] = {
        {
            .forward_coefficient = 1.0f,
            .turn_coefficient_m  = -0.12375f,
        },
        {
            .forward_coefficient = 1.0f,
            .turn_coefficient_m  = 0.12375f,
        }
    };
    static const ChassisInitConfig_t chassis_config = {
        .wheels = chassis_wheels,
        .wheel_kinematics = chassis_wheel_kinematics,
        .wheel_count = 2,
        .auto_start = true
    };
    configASSERT(chassis.init(&chassis, &chassis_config));
}
