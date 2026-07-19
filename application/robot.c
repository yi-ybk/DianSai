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

#include "zhangdatou_42.h"
#include "gimbal.h"

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
static void zdt42Init(void);
static void gimbalInit(void);

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
  .stack_size = 256 * 4,
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

Zdt42_t zdt42_motor_yaw   = { ZDT42_OBJECT_DEFAULT };
Zdt42_t zdt42_motor_pitch = { ZDT42_OBJECT_DEFAULT };

Gimbal_t gimbal = {GIMBAL_OBJECT_DEFAULT};

void robotInit(void)
{

    grayInit();
    keyInit();
    ledInit();
    oledInit();

    encoderInit();
    motorInit();
    pidInit();
    wheelInit();
    chassisInit();

    zdt42Init();

    imu0TaskHandle = osThreadNew(imuParseTask, &imu0, &imu0Task_attributes);
    oledTaskHandle = osThreadNew(oledTask, NULL, &oledTask_attributes);
    testTaskHandle = osThreadNew(testTask, NULL, &testTask_attributes);

}

void testTask(void *argument)
{
    for (;;)
    {

        osDelay(10U);
    }

    
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

        //oled.refresh(&oled);

        //osDelay(50);
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

static void zdt42Init(void){
    const Zdt42InitConfig_t zdt42_motor_yaw_config   = ZDT42_INIT_CONFIG_DEFAULT(&hcan2, 1U);
    const Zdt42InitConfig_t zdt42_motor_pitch_config = ZDT42_INIT_CONFIG_DEFAULT(&hcan2, 2U);

    zdt42_motor_yaw.init(&zdt42_motor_yaw, &zdt42_motor_yaw_config);
    zdt42_motor_pitch.init(&zdt42_motor_pitch, &zdt42_motor_pitch_config);
}

static void gimbalInit(void){
    const GimbalInitConfig_t gimbal_config = {
        .yaw = {
            .motor                = &zdt42_motor_yaw,
            .pulses_per_motor_rev = 3200.0f,
            .motor_to_axis_ratio  = 1.0f,
            .min_angle_deg        = -170.0f,
            .max_angle_deg        = 170.0f,
            .position_offset_deg  = 0.0f,
            .reversed             = false,
            .home_mode            = ZDT42_HOME_NEAREST,
        },
        .pitch = {
            .motor = &zdt42_motor_pitch,
            .pulses_per_motor_rev = 3200.0f,
            .motor_to_axis_ratio  = 1.0f,
            .min_angle_deg        = -45.0f,
            .max_angle_deg        = 90.0f,
            .position_offset_deg  = 0.0f,
            .reversed             = true,
            .home_mode            = ZDT42_HOME_NEAREST,
        },
        .position_speed_rpm  = 60U,
        .acceleration        = 10U,
        .feedback_period_ms  = 20U,
        .feedback_timeout_ms = 100U,
        .auto_enable         = true,
    };

    gimbal.init(&gimbal, &gimbal_config);
}
