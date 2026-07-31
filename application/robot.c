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
#include "ball_control.h"
#include "slaver.h"

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
static float trackLimitForwardSpeed(float requested_speed_mps);
static float trackSmoothForwardSpeed(float target_speed_mps, float dt_s);
static float trackWrapAngleRad(float angle_rad);
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
static void ballControlInit(void);
static void gpioInterruptDispatch(GPIO_TypeDef *GPIOx);

/***********************************/

/******** 线程句柄和属性 ********/
static StaticTask_t chassisControlTaskControlBlock;
static StackType_t chassisControlTaskStack[128];
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

BallControl_t ball_control = { BALL_CONTROL_OBJECT_DEFAULT };
Slaver_t ball_position_receiver = { SLAVER_OBJECT_DEFAULT };
static SlaverSimpleFloatProtocol_t ball_position_protocol;

float ball_target = 0.0f;
float ball_real   = 0.0f;

float target_5cm  = 5.0f;
float target_f5cm = -5.0f;

#define ROBOT_MODE_REQUIREMENT_3          3U
#define ROBOT_MODE_REQUIREMENT_2          2U
#define ROBOT_MODE_REQUIREMENT_4          4U
#define ROBOT_MODE_REQUIREMENT_5          5U
#define ROBOT_MODE_REQUIREMENT_6          6U
#define BALL_SERVO_CENTER_ANGLE_DEG       135.0f
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
#define BALL_HOST_RX_BUFFER_SIZE             64U
#define BALL_CONTROL_TASK_PERIOD_MS          10U
/*
TRACK_LAP_DISTANCE_M：跑一圈的目标距离，当前约 6.1416m。
TRACK_FINISH_ARM_DISTANCE_M：接近一圈末段后，才允许识别 A 点终点黑线，避免刚启动就误判。
TRACK_POSITION_TOLERANCE_M：停车位置允许误差，当前 0.02m，即题目要求的 2cm。
TRACK_POSITION_KP：位置环 P 参数。增大后接近终点时速度更高，但容易冲过；减小后减速更早、更平稳。
TRACK_FINISH_LINE_MIN_BLACK_COUNT：至少多少路灰度传感器检测到黑色，才认为到达终点线。
TRACK_MAX_RUN_TIME_MS：最长运行时间，当前 20000ms。
REQUIREMENT4_AB_DISTANCE_M：要求 4 从 A 到 B 的目标里程，当前 1.5m。
REQUIREMENT4_FORWARD_SPEED_MPS：要求 4 的巡线前进速度，当前 0.35m/s。
REQUIREMENT4_MAX_RUN_TIME_MS：要求 4 的 AB 最大运行时间，当前 8000ms。
*/
volatile uint8_t mode = ROBOT_MODE_REQUIREMENT_2;
volatile uint32_t oled_refresh_error_count = 0U;
volatile uint32_t oled_recovery_count = 0U;
volatile uint32_t ball_control_init_error = 0U;
volatile uint32_t ball_host_init_error = 0U;
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
    ballControlInit();
   
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
    (void)argument;
    while (1)
    {
        motor_ball_servo.set_angle(&motor_ball_servo,45.0f);
        chassis.update(&chassis,0.005f);
        osDelay(5);
    }
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
        menu_active = ball_menu_active;
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
    BallControlData_t ball_data;
    uint32_t display_time_ms;
    bool running;

    taskENTER_CRITICAL();
    running = track_running;
    display_time_ms = running ? track_elapsed_ms : track_total_time_ms;
    taskEXIT_CRITICAL();
    if (mode == ROBOT_MODE_REQUIREMENT_3)
    {
        ball_control.get_data(&ball_control, &ball_data);
    }

    oled.draw_string(&oled, 0, 0, "mode:", OLED_COLOR_WHITE);
    oled.draw_int(&oled, 5, 0, mode, OLED_COLOR_WHITE);
    if (trackModeIsSupported(mode))
    {
        if (mode == ROBOT_MODE_REQUIREMENT_3)
            oled.draw_string(&oled, 0, 2,
                             ball_data.sequence_complete ? "R3:DONE" :
                             (running ? "R3:RUN" : "R3:STOP"),
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
    BallControlData_t control_data;
    float target;
    float real;

    ball_control.get_data(&ball_control, &control_data);
    taskENTER_CRITICAL();
    target = ball_target;
    real = ball_real;
    taskEXIT_CRITICAL();
    if (control_data.state != BALL_CONTROL_IDLE)
        target = control_data.target_cm;

    oled.draw_string(&oled, 0, 0, "BALL MENU", OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 2, "target:", OLED_COLOR_WHITE);
    oled.draw_float(&oled, 8, 2, target, 2, OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 4, "real:", OLED_COLOR_WHITE);
    oled.draw_float(&oled, 8, 4, real, 2, OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 6, "K1 short: save", OLED_COLOR_WHITE);
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
    float requested_target;

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
            taskENTER_CRITICAL();
            ball_real = latest.value;
            taskEXIT_CRITICAL();
            ball_control.set_measurement(&ball_control,
                                         latest.value,
                                         latest.last_update_tick);
        }

        if ((mode == ROBOT_MODE_REQUIREMENT_4) ||
            (mode == ROBOT_MODE_REQUIREMENT_5) ||
            (mode == ROBOT_MODE_REQUIREMENT_6))
        {
            taskENTER_CRITICAL();
            requested_target = ball_target;
            taskEXIT_CRITICAL();
            ball_control.set_target(&ball_control, requested_target);
        }

        ball_control.update(&ball_control, HAL_GetTick());
        osDelay(BALL_CONTROL_TASK_PERIOD_MS);
    }
}

static void trackTask(void *argument)
{
    BallControlData_t ball_data;
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
                trackStart(requested_start_tick);
        }

        if (track_running)
        {
            now_tick = HAL_GetTick();
            if (!track_timer_stopped)
                track_elapsed_ms = now_tick - track_start_tick;
            if ((mode == ROBOT_MODE_REQUIREMENT_3) ||
                (mode == ROBOT_MODE_REQUIREMENT_4) ||
                (mode == ROBOT_MODE_REQUIREMENT_5) ||
                (mode == ROBOT_MODE_REQUIREMENT_6))
            {
                ball_control.get_data(&ball_control, &ball_data);
                if (ball_data.state == BALL_CONTROL_FAULT)
                {
                    track_stop_reason = 4U;
                    trackStop(now_tick);
                    osDelay(5);
                    continue;
                }
                if ((mode == ROBOT_MODE_REQUIREMENT_3) &&
                    ball_data.sequence_complete &&
                    (!track_timer_stopped))
                {
                    taskENTER_CRITICAL();
                    track_elapsed_ms = ball_data.completion_time_ms;
                    track_total_time_ms = ball_data.completion_time_ms;
                    track_timer_stopped = true;
                    taskEXIT_CRITICAL();
                }
                if ((mode != ROBOT_MODE_REQUIREMENT_3) &&
                    ((ball_data.state != BALL_CONTROL_ACTIVE) ||
                     (!ball_data.measurement_valid) ||
                     ball_data.measurement_stale))
                {
                    track_forward_speed_command = 0.0f;
                    chassis.set_velocity(&chassis, 0.0f, 0.0f);
                    osDelay(5);
                    continue;
                }
            }
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
        }
        else
        {
            last_control_tick = 0U;
            chassis.set_velocity(&chassis, 0.0f, 0.0f);
        }
        osDelay(5);
    }
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
    float requested_ball_target = 0.0f;
    bool ball_start_ok = true;

    chassis.set_velocity(&chassis, 0.0f, 0.0f);
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
    if (mode == ROBOT_MODE_REQUIREMENT_3)
    {
        ball_start_ok = ball_control.start(
            &ball_control,
            BALL_PROFILE_REQUIREMENT3,
            0.0f,
            target_5cm,
            target_f5cm,
            start_tick);
    }
    else if ((mode == ROBOT_MODE_REQUIREMENT_4) ||
             (mode == ROBOT_MODE_REQUIREMENT_5) ||
             (mode == ROBOT_MODE_REQUIREMENT_6))
    {
        taskENTER_CRITICAL();
        requested_ball_target = ball_target;
        taskEXIT_CRITICAL();
        ball_start_ok = ball_control.start(
            &ball_control,
            BALL_PROFILE_HOLD,
            requested_ball_target,
            target_5cm,
            target_f5cm,
            start_tick);
    }

    if (!ball_start_ok)
    {
        track_timer_stopped = true;
        ball_control_init_error = 2U;
        led_red.off(&led_red);
        return;
    }

    track_running = true;
    led_red.on(&led_red);
}

static void trackStop(uint32_t stop_tick)
{
    ChassisData_t chassis_data;

    chassis.set_velocity(&chassis, 0.0f, 0.0f);
    chassis.get_data(&chassis, &chassis_data);
    track_stop_distance_m = chassis_data.distance_m;
    if ((mode == ROBOT_MODE_REQUIREMENT_3) ||
        (mode == ROBOT_MODE_REQUIREMENT_4) ||
        (mode == ROBOT_MODE_REQUIREMENT_5) ||
        (mode == ROBOT_MODE_REQUIREMENT_6))
    {
        ball_control.stop(&ball_control);
    }
    track.pid.reset(&track.pid);
    track_forward_speed_command = 0.0f;
    taskENTER_CRITICAL();
    if (!track_timer_stopped)
        track_elapsed_ms = stop_tick - track_start_tick;
    track_total_time_ms = track_elapsed_ms;
    track_timer_stopped = true;
    taskEXIT_CRITICAL();
    track_running = false;
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
    led_green.toggle(&led_green);
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

static void ballControlInit(void)
{
    const BallControlInitConfig_t ball_control_config = {
        .motor = &motor_ball_servo,
        .position_pid = {
            .kp = 0.8f,
            .ki = 0.0f,
            .kd = 0.08f,
            .enable_output_limit = true,
            .output_min = -5.0f,
            .output_max = 5.0f,
            .enable_integral_limit = true,
            .integral_min = -1.0f,
            .integral_max = 1.0f,
            .deadband = 0.15f,
            .derivative_filter_tau_s = 0.08f,
            .derivative_on_measurement = true,
            .reset_integral_on_deadband = true,
        },
        .motor_to_rod_ratio = 1.0f,
        .motor_direction = 1.0f,
        .servo_center_angle_deg = BALL_SERVO_CENTER_ANGLE_DEG,
        .maximum_target_cm = 12.0f,
        .maximum_rod_angle_deg = 5.0f,
        .maximum_servo_slew_deg_per_s = 120.0f,
        .measurement_velocity_filter_s = 0.08f,
        .prediction_horizon_s = 0.0f,
        .sequence_tolerance_cm = 0.8f,
        .control_period_ms = 10U,
        .motor_command_period_ms = 20U,
        .measurement_timeout_ms = 200U,
        .sequence_hold_ms = 200U,
    };

    if (!ball_control.init(&ball_control, &ball_control_config))
    {
        ball_control_init_error = 1U;
        configASSERT(false);
    }
}
