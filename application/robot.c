#include "robot.h"
#include "usart.h"

#include "imu_driver.h"
#include "mg354pdh0_driver.h"

#include "gray.h"

#include "led_driver.h"

#include "key_driver.h"

#include "oled_driver.h"

#include "encoder.h"
#include "motor_driver.h"
#include "pid.h"
#include "chassis.h"

static void imuParseTask(void *argument);
static void testTask(void *argument);
static void oledTask(void *argument);

static void keyEventCallback(Key_t *key, KeyEvent_t event, void *context);

static void grayInit(void);
static void keyInit(void);
static void ledInit(void);
static void oledInit(void);
static void encoderInit(void);
static void motorInit(void);
static void pidInit(void);
static void wheelInit(void);
static void chassisInit(void);
static void chassisTestUpdateTelemetry(uint32_t phase_index,
                                       float phase_elapsed_s,
                                       float command_forward_speed_mps,
                                       float command_turn_speed_radps);

typedef struct
{
    float duration_s;
    float forward_speed_mps;
    float turn_speed_radps;
} ChassisTestPhase_t;

typedef struct ChassisTestTelemetry
{
    uint32_t phase_index;
    float phase_elapsed_s;
    float control_dt_s;
    float command_forward_speed_mps;
    float command_turn_speed_radps;

    float forward_speed_mps;
    float turn_speed_radps;
    int32_t encoder_left_count;
    int32_t encoder_left_delta;
    float encoder_left_speed_cps;
    int32_t encoder_right_count;
    int32_t encoder_right_delta;
    float encoder_right_speed_cps;
    float wheel_left_target_speed_mps;
    float wheel_left_speed_mps;
    float wheel_right_target_speed_mps;
    float wheel_right_speed_mps;
    float motor_left_output;
    float motor_right_output;
    float wheel_left_pid_error;
    float wheel_left_pid_p_out;
    float wheel_left_pid_i_out;
    float wheel_left_pid_d_out;
    uint8_t wheel_left_pid_in_deadband;
    float wheel_right_pid_error;
    float wheel_right_pid_p_out;
    float wheel_right_pid_i_out;
    float wheel_right_pid_d_out;
    uint8_t wheel_right_pid_in_deadband;
    float imu_gyro_z_radps;
    float imu_yaw_deg;
} ChassisTestTelemetry_t;

static const ChassisTestPhase_t chassis_test_phases[] = {
    { 5.0f,  0.0f,  0.0f },
    { 10.0f, 0.1f,  0.0f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.2f,  0.0f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.3f,  0.0f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.0f,  0.1f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.0f, -0.1f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.0f,  0.2f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.0f, -0.2f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.0f,  0.4f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.0f, -0.4f },
    { 3.0f,  0.0f,  0.0f },
    { 15.0f, 0.3f,  0.2f },
    { 3.0f,  0.0f,  0.0f },
    { 10.0f, 0.3f,  0.0f },
    { 5.0f,  0.0f,  0.0f },
};

osThreadId_t imu0TaskHandle;
const osThreadAttr_t imu0Task_attributes = {
  .name = "imu0Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 2048,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t testTaskHandle;
const osThreadAttr_t testTask_attributes = {
  .name = "testTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

static ImuMahonyConfig_t imu0_mahony_config = {
    .sample_period_s = 0.008f,
    .proportional_gain = 0.5f,
    .integral_gain = 0.0f,
    .initial_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
    .accel_gravity_sign = -1.0f,
};

static Mg354pdh0GyroCalibration_t imu0_gyro_calibration =
    MG354PDH0_GYRO_CALIBRATION_DEFAULT(250U);

Imu_t imu0 = {
        IMU_OBJECT_DEFAULT,

        .attitude_solver = {
            .init    = ImuMahonySolverInit,
            .update  = ImuMahonySolverUpdate,
            .context = &imu0_mahony_config,
        },

        .protocol_frame_len    = MG354PDH0_FRAME_LEN,
        .protocol_header_bytes = (const uint8_t[]){MG354PDH0_FRAME_HEADER},
        .protocol_header_len   = sizeof((const uint8_t[]){MG354PDH0_FRAME_HEADER}),
        .protocol_tail_bytes   = (const uint8_t[]){MG354PDH0_FRAME_TAIL},
        .protocol_tail_len     = sizeof((const uint8_t[]){MG354PDH0_FRAME_TAIL}),

        .init_config = {
            .recv_buff_size = IMU_UART_DMA_RX_BUFFER_LEN,
            .usart_handle   = &huart1,
            .parser         = Mg354pdh0FrameParse,
            .parser_context = &imu0_gyro_calibration,
            .device_init    = Mg354pdh0DeviceInit,
            .device_context = NULL,
        },
};

Gray_t gray = { GRAY_OBJECT_DEFAULT };

Led_t green_led = { LED_OBJECT_DEFAULT  };

Key_t key1 = { KEY_OBJECT_DEFAULT };

Encoder_t encoder_left  = { ENCODER_OBJECT_DEFAULT };
Encoder_t encoder_right = { ENCODER_OBJECT_DEFAULT };

Motor_t motor_left  = { MOTOR_OBJECT_DEFAULT };
Motor_t motor_right = { MOTOR_OBJECT_DEFAULT };

Pid_t wheel_left_pid  = { PID_OBJECT_DEFAULT };
Pid_t wheel_right_pid = { PID_OBJECT_DEFAULT };

Wheel_t wheel_left  = { WHEEL_OBJECT_DEFAULT };
Wheel_t wheel_right = { WHEEL_OBJECT_DEFAULT };

Chassis_t chassis = { CHASSIS_OBJECT_DEFAULT };

volatile ChassisTestTelemetry_t chassis_test_telemetry = {
    .phase_index = UINT32_MAX,
};

void robotInit(void)
{
    __disable_irq();

    grayInit();
    keyInit();
    ledInit();
    oledInit();

    encoderInit();
    motorInit();
    pidInit();
    wheelInit();
    chassisInit();

    imu0TaskHandle = osThreadNew(imuParseTask, &imu0, &imu0Task_attributes);
    oledTaskHandle = osThreadNew(oledTask, NULL, &oledTask_attributes);
    testTaskHandle = osThreadNew(testTask, NULL, &testTask_attributes);

    __enable_irq();
}

float dt_s;
void testTask(void *argument)
{
    uint32_t last_tick;
    uint32_t next_tick;
    uint32_t now_tick;
    uint32_t tick_freq;
    // float dt_s;
    uint32_t phase_index = 0U;
    float phase_elapsed_s = 0.0f;
    const uint32_t phase_count = sizeof(chassis_test_phases) /
                                 sizeof(chassis_test_phases[0]);

    (void)argument;

    chassis.set_velocity(&chassis, 0.0f, 0.0f);
    chassis.reset_odometry(&chassis);

    tick_freq = osKernelGetTickFreq();
    last_tick = osKernelGetTickCount();
    next_tick = last_tick;

    

    for (;;)
    {

        osDelay(10); 

        // next_tick += 10U; // 10ms周期

        // if (osDelayUntil(next_tick) != osOK)
        //     next_tick = osKernelGetTickCount();

        // now_tick     = osKernelGetTickCount();
        // dt_s         = (float)(now_tick - last_tick) / (float)tick_freq;
        // last_tick    = now_tick;

        // phase_elapsed_s += dt_s;
        // while ((phase_index + 1U < phase_count) &&
        //        (phase_elapsed_s >= chassis_test_phases[phase_index].duration_s))
        // {
        //     phase_elapsed_s -= chassis_test_phases[phase_index].duration_s;
        //     phase_index++;
        //     chassis.set_velocity(&chassis,
        //                          chassis_test_phases[phase_index].forward_speed_mps,
        //                          chassis_test_phases[phase_index].turn_speed_radps);
        // }

        // if ((phase_index + 1U == phase_count) &&
        //     (phase_elapsed_s > chassis_test_phases[phase_index].duration_s))
        // {
        //     phase_elapsed_s = chassis_test_phases[phase_index].duration_s;
        // }

        // chassis.update(&chassis, dt_s);
        // chassisTestUpdateTelemetry(phase_index,
        //                            phase_elapsed_s,
        //                            chassis_test_phases[phase_index].forward_speed_mps,
        //                            chassis_test_phases[phase_index].turn_speed_radps);
    }
}

static void chassisTestUpdateTelemetry(uint32_t phase_index,
                                       float phase_elapsed_s,
                                       float command_forward_speed_mps,
                                       float command_turn_speed_radps)
{
    ImuData_t imu_data;

    imu0.get_data(&imu0, &imu_data);

    chassis_test_telemetry.phase_index = phase_index;
    chassis_test_telemetry.phase_elapsed_s = phase_elapsed_s;
    chassis_test_telemetry.control_dt_s = dt_s;
    chassis_test_telemetry.command_forward_speed_mps = command_forward_speed_mps;
    chassis_test_telemetry.command_turn_speed_radps = command_turn_speed_radps;
    chassis_test_telemetry.forward_speed_mps = chassis.data.forward_speed_mps;
    chassis_test_telemetry.turn_speed_radps = chassis.data.turn_speed_radps;
    chassis_test_telemetry.encoder_left_count = encoder_left.data.count;
    chassis_test_telemetry.encoder_left_delta = encoder_left.data.delta;
    chassis_test_telemetry.encoder_left_speed_cps = encoder_left.data.speed_cps;
    chassis_test_telemetry.encoder_right_count = encoder_right.data.count;
    chassis_test_telemetry.encoder_right_delta = encoder_right.data.delta;
    chassis_test_telemetry.encoder_right_speed_cps = encoder_right.data.speed_cps;
    chassis_test_telemetry.wheel_left_target_speed_mps = wheel_left.data.target_linear_speed_mps;
    chassis_test_telemetry.wheel_left_speed_mps = wheel_left.data.linear_speed_mps;
    chassis_test_telemetry.wheel_right_target_speed_mps = wheel_right.data.target_linear_speed_mps;
    chassis_test_telemetry.wheel_right_speed_mps = wheel_right.data.linear_speed_mps;
    chassis_test_telemetry.motor_left_output = motor_left.data.output;
    chassis_test_telemetry.motor_right_output = motor_right.data.output;
    chassis_test_telemetry.wheel_left_pid_error = wheel_left_pid.data.error;
    chassis_test_telemetry.wheel_left_pid_p_out = wheel_left_pid.data.p_out;
    chassis_test_telemetry.wheel_left_pid_i_out = wheel_left_pid.data.i_out;
    chassis_test_telemetry.wheel_left_pid_d_out = wheel_left_pid.data.d_out;
    chassis_test_telemetry.wheel_left_pid_in_deadband =
        wheel_left_pid.data.in_deadband ? 1U : 0U;
    chassis_test_telemetry.wheel_right_pid_error = wheel_right_pid.data.error;
    chassis_test_telemetry.wheel_right_pid_p_out = wheel_right_pid.data.p_out;
    chassis_test_telemetry.wheel_right_pid_i_out = wheel_right_pid.data.i_out;
    chassis_test_telemetry.wheel_right_pid_d_out = wheel_right_pid.data.d_out;
    chassis_test_telemetry.wheel_right_pid_in_deadband =
        wheel_right_pid.data.in_deadband ? 1U : 0U;
    chassis_test_telemetry.imu_gyro_z_radps = imu_data.gyro.z;
    chassis_test_telemetry.imu_yaw_deg = imu_data.angle.z;
}

void imuParseTask(void *argument)
{
    Imu_t *imu = (Imu_t *)argument;

    if (imu == NULL)
        while (1);

    if ((imu->init == NULL) || !imu->init(imu, &imu->init_config))
        osThreadExit();

    for (;;)
    {
        if (imu->process != NULL)
            imu->process(imu);

        osDelay(6);
    }
}


Oled_t oled = { OLED_OBJECT_DEFAULT };
void oledTask(void *argument)
{
    (void)argument;

    // oled.draw_string(&oled, 0, 0,  "acc:", OLED_COLOR_WHITE);
    // oled.draw_string(&oled, 0, 1,  "x:"  , OLED_COLOR_WHITE);
    // oled.draw_string(&oled, 0, 2,  "y:"  , OLED_COLOR_WHITE);
    // oled.draw_string(&oled, 0, 3,  "z:"  , OLED_COLOR_WHITE);

    // oled.draw_string(&oled, 10, 0, "angle:", OLED_COLOR_WHITE);
    // oled.draw_string(&oled, 10, 1, "x:"    , OLED_COLOR_WHITE);
    // oled.draw_string(&oled, 10, 2, "y:"    , OLED_COLOR_WHITE);
    // oled.draw_string(&oled, 10, 3, "z:"    , OLED_COLOR_WHITE);

    // oled.draw_string(&oled, 0, 0,  "cha_for:", OLED_COLOR_WHITE);

    // oled.draw_string(&oled, 0, 3,  "cha_turn:", OLED_COLOR_WHITE);

    // oled.refresh(&oled);

    for (;;)
    {
        // ImuData_t imu_data;
        // imu0.get_data(&imu0, &imu_data);

        // oled.draw_float(&oled, 3, 1, imu_data.accel.x, 2, OLED_COLOR_WHITE);
        // oled.draw_float(&oled, 3, 2, imu_data.accel.y, 2, OLED_COLOR_WHITE);
        // oled.draw_float(&oled, 3, 3, imu_data.accel.z, 2, OLED_COLOR_WHITE);

        // oled.draw_float(&oled, 13, 1, imu_data.angle.x, 2, OLED_COLOR_WHITE);
        // oled.draw_float(&oled, 13, 2, imu_data.angle.y, 2, OLED_COLOR_WHITE);
        // oled.draw_float(&oled, 13, 3, imu_data.angle.z, 2, OLED_COLOR_WHITE);

        // ChassisData_t chassis_data;
        // chassis.get_data(&chassis, &chassis_data);

        // oled.draw_float(&oled, 11, 0, chassis_data.target_forward_speed_mps, 2, OLED_COLOR_WHITE);
        // oled.draw_float(&oled, 11, 3, chassis_data.target_turn_speed_radps, 2, OLED_COLOR_WHITE);
        // oled.draw_float(&oled, 11, 1, chassis_data.forward_speed_mps, 2, OLED_COLOR_WHITE);
        // oled.draw_float(&oled, 11, 4, chassis_data.turn_speed_radps, 2, OLED_COLOR_WHITE);

        gray.update(&gray);

        oled.draw_int(&oled, 0, 1, gray.read_channel(&gray, 0U), OLED_COLOR_WHITE);
        oled.draw_int(&oled, 0, 2, gray.read_channel(&gray, 1U), OLED_COLOR_WHITE);
        oled.draw_int(&oled, 0, 3, gray.read_channel(&gray, 2U), OLED_COLOR_WHITE);
        oled.draw_int(&oled, 0, 4, gray.read_channel(&gray, 3U), OLED_COLOR_WHITE);

        oled.refresh(&oled);

        osDelay(50);
    }
}

void keyEventCallback(Key_t *key, KeyEvent_t event, void *context)
{
    (void)context;

    if(key == &key1)
    {
        if (event == KEY_EVENT_PRESS)
        {
            green_led.toggle(&green_led);
        }
    }
}

static void grayInit(void){
    const GrayChannelConfig_t gray_channels[] = {
        { .GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_0 },
        { .GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_1 },
        { .GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_2 },
        { .GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_3 },
    };

    const GrayInitConfig_t gray_config = {
        .channels      = gray_channels,
        .channel_count = sizeof(gray_channels) / sizeof(gray_channels[0]),
        .black_state   = GPIO_PIN_RESET,
    };

    gray.init(&gray, &gray_config);
}

static void keyInit(void)
{
    const KeyInitConfig_t key1_config = {
        .GPIOx          = GPIOA,
        .GPIO_Pin       = GPIO_PIN_0,
        .active_state   = GPIO_PIN_SET,
        .exti_mode      = GPIO_EXTI_MODE_RISING_FALLING,
        .debounce_ms    = 50,
        .event_callback = keyEventCallback,
        .event_context  = NULL,
    };
    key1.init(&key1, &key1_config);
}

static void ledInit(void)
{
    const LedInitConfig_t green_led_config = {
        .GPIOx        = GPIOB,
        .GPIO_Pin     = GPIO_PIN_2,
        .active_state = GPIO_PIN_SET,
        .init_state   = LED_STATE_OFF,
        .id           = "Green LED",
    };
    green_led.init(&green_led, &green_led_config);
}

static void oledInit(void)
{
    const OledInitConfig_t oled_config = {
        .i2c_handle = NULL,
        .iic_bus_mode = IIC_BUS_SOFTWARE,
        .soft_iic = {
            .scl = {
                .GPIOx    = GPIOA,
                .GPIO_Pin = GPIO_PIN_3,
            },
            .sda = {
                .GPIOx    = GPIOA,
                .GPIO_Pin = GPIO_PIN_4,
            },
            .delay_us = 2U,
        },
    };
    oled.init(&oled, &oled_config);
}

static void encoderInit(void)
{
    const EncoderInitConfig_t encoder_left_config = {
        .htim           = &htim3,
        .channel        = TIM_CHANNEL_ALL,
        .reversed       = true,
        .counts_per_rev = 13.0f * 4.0f,
        .speed_window_samples = 5U,
        .auto_start     = true,
    };
    const EncoderInitConfig_t encoder_right_config = {
        .htim           = &htim4,
        .channel        = TIM_CHANNEL_ALL,
        .reversed       = false,
        .counts_per_rev = 13.0f * 4.0f,
        .speed_window_samples = 5U,
        .auto_start     = true,
    };
    encoder_left.init(&encoder_left, &encoder_left_config);
    encoder_right.init(&encoder_right, &encoder_right_config);
}

static void motorInit(void)
{
    const MotorInitConfig_t motor_left_config = {
        .type            = MOTOR_TYPE_REDUCTION,
        .use_reverse_pwm = false,
        .pwm = {
            .htim      = &htim5,
            .channel   = TIM_CHANNEL_2,
            .period    = 0.001f,
            .init_duty = 0.0f,
        },
        .use_tb6612 = true,
        .tb6612 = {
            .in1 = {
                .GPIOx    = GPIOC,
                .GPIO_Pin = GPIO_PIN_9,
            },
            .in2 = {
                .GPIOx    = GPIOD,
                .GPIO_Pin = GPIO_PIN_3,
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
            .htim      = &htim5,
            .channel   = TIM_CHANNEL_3,
            .period    = 0.001f,
            .init_duty = 0.0f,
        },
        .use_tb6612 = true,
        .tb6612 = {
            .in1 = {
                .GPIOx    = GPIOC,
                .GPIO_Pin = GPIO_PIN_12,
            },
            .in2 = {
                .GPIOx    = GPIOC,
                .GPIO_Pin = GPIO_PIN_8,
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
    motor_left.init(&motor_left  , &motor_left_config);
    motor_right.init(&motor_right, &motor_right_config);
}

static void pidInit(void)
{
    const PidInitConfig_t wheel_left_pid_config = {
        .kp = 2.5f,
        .ki = 1.0f,
        .kd = 0.0f,
        .enable_output_limit = true,
        .output_min = -1.0f,
        .output_max = 1.0f,
        .enable_integral_limit = true,
        .integral_min = -0.5f,
        .integral_max = 0.5f,
        .deadband     = 0.003f,
        .derivative_on_measurement  = true,
        .reset_integral_on_deadband = false,
    };
    const PidInitConfig_t wheel_right_pid_config = {
        .kp = 2.5f,
        .ki = 1.0f,
        .kd = 0.0f,
        .enable_output_limit = true,
        .output_min = -1.0f,
        .output_max = 1.0f,
        .enable_integral_limit = true,
        .integral_min = -0.5f,
        .integral_max = 0.5f,
        .deadband     = 0.003f,
        .derivative_on_measurement  = true,
        .reset_integral_on_deadband = false,
    };
    wheel_left_pid.init(&wheel_left_pid, &wheel_left_pid_config);
    wheel_right_pid.init(&wheel_right_pid, &wheel_right_pid_config);
}

static void wheelInit(void)
{
    const WheelInitConfig_t wheel_left_config = {
        .motor = &motor_left,
        .speed_pid = &wheel_left_pid,
        .use_speed_pid = true,
        .radius_m = 0.0325f,
        .encoder_to_wheel_ratio = 28.0f,
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
        .radius_m = 0.03387f,
        .encoder_to_wheel_ratio = 28.0f,
        .max_linear_speed_mps   = 1.2f,
        .output_min = -1.0f,
        .output_max = 1.0f,
        .speed_pid_no_reverse = true,
        .reversed = false,
        .auto_start = false,
    };
    wheel_left.init(&wheel_left, &wheel_left_config);
    wheel_right.init(&wheel_right, &wheel_right_config);
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
        .imu = NULL,
        .auto_start = true
    };
    chassis.init(&chassis, &chassis_config);
}
