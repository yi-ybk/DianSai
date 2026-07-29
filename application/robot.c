#include "robot.h"
#include "cmsis_os.h"
#include "ti_msp_dl_config.h"
#include "bsp_can.h"

#include "led_driver.h"

#include "oled_driver.h"

#include "key_driver.h"

#include "gray.h"
#include "track.h"

#include "encoder.h"
#include "motor_driver.h"
#include "pid.h"
#include "chassis.h"

#include "imu_driver.h"
#include "mg354pdh0_driver.h"

/************ 函数声明 **************/
static void testTask(void *argument);
static void oledTask(void *argument);
static void imuParseTask(void *argument);
static void trackTask(void *argument);

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
/***********************************/

/******** 线程句柄和属性 ********/
static StaticTask_t testTaskControlBlock;
static StackType_t testTaskStack[128];
osThreadId_t testTaskHandle;
const osThreadAttr_t testTask_attributes = {
  .name = "testTask",
  .cb_mem = &testTaskControlBlock,
  .cb_size = sizeof(testTaskControlBlock),
  .stack_mem = testTaskStack,
  .stack_size = sizeof(testTaskStack),
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
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t imu0TaskHandle;
const osThreadAttr_t imu0Task_attributes = {
  .name = "imu0Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t trackTaskHandle;
const osThreadAttr_t trackTask_attributes = {
  .name = "trackTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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

TIM_HandleTypeDef htim_motor_pwm = {
    .Instance = PWM_MOTOR_INST,
    .Init = {
        .Prescaler = 0U,
        .Period = 3999U,
    },
    .clock_hz = PWM_MOTOR_INST_CLK_FREQ,
};

static UART_HandleTypeDef huart_imu0 = {
    .Instance = UART_IMU0_INST,
};

Pid_t wheel_left_pid  = { PID_OBJECT_DEFAULT };
Pid_t wheel_right_pid = { PID_OBJECT_DEFAULT };

Wheel_t wheel_left  = { WHEEL_OBJECT_DEFAULT };
Wheel_t wheel_right = { WHEEL_OBJECT_DEFAULT };

Chassis_t chassis = { CHASSIS_OBJECT_DEFAULT };

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
            .usart_handle   = &huart_imu0,
            .parser         = Mg354pdh0FrameParse,
            .parser_context = &imu0_gyro_calibration,
            .device_init    = Mg354pdh0DeviceInit,
            .device_context = NULL,
        },
};
/*******************************/

void robotInit(void)
{
    /* 初始化机器人相关的硬件和软件组件 */
    ledInit();
    //buzzerInit();
    oledInit();
    keyInit();
    grayInit();
    trackInit();
    encoderInit();
    motorInit();
    pidInit();
    wheelInit();
    chassisInit();
    testTaskHandle = osThreadNew(testTask, NULL, &testTask_attributes);
    oledTaskHandle = osThreadNew(oledTask, NULL, &oledTask_attributes);
    imu0TaskHandle = osThreadNew(imuParseTask, &imu0, &imu0Task_attributes);
    trackTaskHandle = osThreadNew(trackTask, NULL, &trackTask_attributes);
    configASSERT(testTaskHandle != NULL);
    configASSERT(oledTaskHandle != NULL);
    configASSERT(imu0TaskHandle != NULL);
    configASSERT(trackTaskHandle != NULL);
}

static void testTask(void *argument)
{
    (void)argument;

    //chassis.set_velocity(&chassis, 0.03f, 0.00f);
    while (1)
    {
 
        chassis.update(&chassis,0.005f);
        osDelay(5);
    }
}

void oledTask(void *argument)
{
    (void)argument;
    oled.draw_string(&oled, 0, 0,  "acc:", OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 1,  "x:"  , OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 2,  "y:"  , OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 3,  "z:"  , OLED_COLOR_WHITE);

    oled.draw_string(&oled, 10, 0, "angle:", OLED_COLOR_WHITE);
    oled.draw_string(&oled, 10, 1, "x:"    , OLED_COLOR_WHITE);
    oled.draw_string(&oled, 10, 2, "y:"    , OLED_COLOR_WHITE);
    oled.draw_string(&oled, 10, 3, "z:"    , OLED_COLOR_WHITE);

    int32_t i = 0;

    while (1)
    {

        ImuData_t imu_data;
        imu0.get_data(&imu0, &imu_data);

        oled.draw_float(&oled, 3, 1, imu_data.accel.x, 2, OLED_COLOR_WHITE);
        oled.draw_float(&oled, 3, 2, imu_data.accel.y, 2, OLED_COLOR_WHITE);
        oled.draw_float(&oled, 3, 3, imu_data.accel.z, 2, OLED_COLOR_WHITE);

        oled.draw_float(&oled, 13, 1, imu_data.angle.x, 2, OLED_COLOR_WHITE);
        oled.draw_float(&oled, 13, 2, imu_data.angle.y, 2, OLED_COLOR_WHITE);
        oled.draw_float(&oled, 13, 3, imu_data.angle.z, 2, OLED_COLOR_WHITE);

        oled.draw_int(&oled, 0, 4, i++, OLED_COLOR_WHITE);

        // oled.draw_string(&oled, 0, 0, "Hello, OLED!", OLED_COLOR_WHITE);

        // gray.update(&gray);
        // oled.draw_int(&oled, 0, 1, gray.read_channel(&gray, 0U), OLED_COLOR_WHITE);
        // oled.draw_int(&oled, 0, 2, gray.read_channel(&gray, 1U), OLED_COLOR_WHITE);
        // oled.draw_int(&oled, 0, 3, gray.read_channel(&gray, 2U), OLED_COLOR_WHITE);
        // oled.draw_int(&oled, 0, 4, gray.read_channel(&gray, 3U), OLED_COLOR_WHITE);

        // encoder_left.update(&encoder_left, 0.05f);
        // encoder_right.update(&encoder_right, 0.05f);

        // oled.draw_int(&oled, 0, 0, encoder_left.get_count(&encoder_left), OLED_COLOR_WHITE);
        // oled.draw_int(&oled, 0, 1, encoder_right.get_count(&encoder_right), OLED_COLOR_WHITE);



        oled.refresh(&oled);
        osDelay(50);
    }
}

static void imuParseTask(void *argument)
{
    Imu_t *imu = (Imu_t *)argument;
    const USART_DMA_Config_s dma_config = {
        .dma        = DMA,
        .rx_channel = DMA_CH_IMU0_RX_CHAN_ID,
        .tx_channel = USART_DMA_CHANNEL_INVALID,
    };

    if (imu == NULL)
    {
        configASSERT(0);
        vTaskDelete(NULL);
        return;
    }

    if ((imu->init == NULL) || !imu->init(imu, &imu->init_config))
    {
        configASSERT(0);
        vTaskDelete(NULL);
        return;
    }
    if (!USARTConfigureDMA(imu->usart, &dma_config) ||
        !USARTStartReceiveDMA(imu->usart))
    {
        configASSERT(0);
        vTaskDelete(NULL);
        return;
    }

    NVIC_ClearPendingIRQ(UART_IMU0_INST_INT_IRQN);
    NVIC_SetPriority(UART_IMU0_INST_INT_IRQN, 2U);
    NVIC_EnableIRQ(UART_IMU0_INST_INT_IRQN);
    for (;;)
    {
        if (imu->process != NULL)
            imu->process(imu);
        osDelay(6);
    }
}

static void trackTask(void *argument)
{
    (void)argument;
    float turn_speed;

    for (;;)
    {
        turn_speed = track.update(&track, 0.02f);
        chassis.set_velocity(&chassis, 0.1, -turn_speed);
        osDelay(20);
    }
}

void UART_IMU0_INST_IRQHandler(void)
{
    USARTIRQHandler(&huart_imu0);
}

void MCAN_GIMBAL_INST_IRQHandler(void)
{
    CANIRQHandler(MCAN_GIMBAL_INST, DL_MCAN_RX_FIFO_NUM_0);
}

void keyEventCallback(Key_t *key, KeyEvent_t event, void *context)
{
    (void)context;

    if(key == &key1)
    {
        if (event == KEY_EVENT_PRESS)
        {
            led_green.toggle(&led_green);
        }
    }
    else if(key == &key2)
    {
        if (event == KEY_EVENT_PRESS)
        {
            led_red.toggle(&led_red);
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
        .base_speed = 0.5f,
        .max_turn_speed = 2.0f,
        .pid_config = {
            .kp = 0.3f,
            .ki = 0.0f,
            .kd = 0.0f,
            .enable_output_limit = true,
            .output_min = -2.0f,
            .output_max = 2.0f,
            .enable_integral_limit = true,
            .integral_min = -1.0f,
            .integral_max = 1.0f,
            .deadband = 0.0f,
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
    configASSERT(motor_left.init(&motor_left, &motor_left_config));
    configASSERT(motor_right.init(&motor_right, &motor_right_config));
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
        .imu = NULL,
        .auto_start = true
    };
    configASSERT(chassis.init(&chassis, &chassis_config));
}
