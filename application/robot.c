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
static void chassisControlTask(void *argument);
static void oledTask(void *argument);
static void imuParseTask(void *argument);
static void trackTask(void *argument);

static void key1ProcessEvents(void);
static void oledDrawDefaultPage(void);
static void oledDrawBallPage(void);
static void trackStart(uint32_t start_tick);
static void trackStop(uint32_t stop_tick);
static float trackCalculateForwardSpeed(float remaining_distance_m);
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

float ball_target = 0.0f;
float ball_real   = 10.0f;

#define ROBOT_MODE_TRACK                  2U
#define TRACK_LAP_DISTANCE_M              (3.0f + 3.14159265f)
#define TRACK_FINISH_ARM_DISTANCE_M       5.5f
#define TRACK_POSITION_TOLERANCE_M        0.02f
#define TRACK_POSITION_KP                 1.0f
#define TRACK_FINISH_LINE_MIN_BLACK_COUNT 5U
#define TRACK_MAX_RUN_TIME_MS             20000U

volatile uint8_t mode = ROBOT_MODE_TRACK;
volatile uint32_t oled_refresh_error_count = 0U;
volatile uint32_t oled_recovery_count = 0U;

static volatile bool ball_menu_active = false;
static volatile bool track_running = false;
static volatile bool track_start_requested = false;
static volatile uint32_t track_start_request_tick = 0U;
static volatile uint32_t track_start_tick = 0U;
static volatile uint32_t track_elapsed_ms = 0U;
static volatile uint32_t track_total_time_ms = 0U;
static bool key1_sample_pressed = false;
static bool key1_stable_pressed = false;
static bool key1_long_press_handled = false;
static uint32_t key1_sample_change_tick = 0U;
static uint32_t key1_press_tick = 0U;
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
    chassisControlTaskHandle = osThreadNew(chassisControlTask, NULL, &chassisControlTask_attributes);
    oledTaskHandle = osThreadNew(oledTask, NULL, &oledTask_attributes);
    imu0TaskHandle = osThreadNew(imuParseTask, &imu0, &imu0Task_attributes);
    trackTaskHandle = osThreadNew(trackTask, NULL, &trackTask_attributes);
    configASSERT(chassisControlTaskHandle != NULL);
    configASSERT(oledTaskHandle != NULL);
    configASSERT(imu0TaskHandle != NULL);
    configASSERT(trackTaskHandle != NULL);
}

static void chassisControlTask(void *argument)
{
    (void)argument;

    //chassis.set_velocity(&chassis, 0.03f, 0.00f);
    while (1)
    {
 
        chassis.update(&chassis,0.005f);
        osDelay(5);
    }
}

static void oledTask(void *argument)
{
    bool menu_active;
    bool refresh_ok;
    uint32_t last_recovery_tick;

    (void)argument;
    last_recovery_tick = HAL_GetTick();

    while (1)
    {
        key1ProcessEvents();
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
            last_recovery_tick = HAL_GetTick();
        }
        else if ((HAL_GetTick() - last_recovery_tick) >= 5000U)
        {
            if (oled.recover(&oled))
                oled_recovery_count++;
            else
                oled_refresh_error_count++;
            last_recovery_tick = HAL_GetTick();
        }

        osDelay(50);
    }
}

static void oledDrawDefaultPage(void)
{
    ImuData_t imu_data;
    uint32_t display_time_ms;
    bool running;

    imu0.get_data(&imu0, &imu_data);
    taskENTER_CRITICAL();
    running = track_running;
    display_time_ms = running ? track_elapsed_ms : track_total_time_ms;
    taskEXIT_CRITICAL();

    oled.draw_string(&oled, 0, 0, "mode:", OLED_COLOR_WHITE);
    oled.draw_int(&oled, 5, 0, mode, OLED_COLOR_WHITE);
    if (mode == ROBOT_MODE_TRACK)
    {
        oled.draw_string(&oled, 0, 2,
                         running ? "track:RUN" : "track:STOP",
                         OLED_COLOR_WHITE);
        oled.draw_string(&oled, 0, 4, "time:", OLED_COLOR_WHITE);
        oled.draw_float(&oled, 5, 4,
                        (float)display_time_ms * 0.001f,
                        2, OLED_COLOR_WHITE);
        oled.draw_string(&oled, 11, 4, "s", OLED_COLOR_WHITE);
    }
}

static void oledDrawBallPage(void)
{
    float target;
    float real;

    taskENTER_CRITICAL();
    target = ball_target;
    real = ball_real;
    taskEXIT_CRITICAL();

    oled.draw_string(&oled, 0, 0, "BALL MENU", OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 2, "target:", OLED_COLOR_WHITE);
    oled.draw_float(&oled, 8, 2, target, 2, OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 4, "real:", OLED_COLOR_WHITE);
    oled.draw_float(&oled, 8, 4, real, 2, OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 6, "K1 short: save", OLED_COLOR_WHITE);
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
    ChassisData_t chassis_data;
    uint32_t now_tick;
    uint32_t requested_start_tick;
    float distance_m;
    float remaining_distance_m;
    float forward_speed;
    (void)argument;
    float turn_speed;
    bool start_requested;
    bool finish_line_detected;

    for (;;)
    {
        start_requested = false;
        requested_start_tick = 0U;
        if (mode == ROBOT_MODE_TRACK)
        {
            taskENTER_CRITICAL();
            if (track_start_requested)
            {
                start_requested = true;
                requested_start_tick = track_start_request_tick;
                track_start_requested = false;
            }
            taskEXIT_CRITICAL();

            if (start_requested && !track_running)
                trackStart(requested_start_tick);
        }

        if ((mode == ROBOT_MODE_TRACK) && track_running)
        {
            now_tick = HAL_GetTick();
            track_elapsed_ms = now_tick - track_start_tick;
            turn_speed = track.update(&track, 0.02f);
            chassis.get_data(&chassis, &chassis_data);
            distance_m = (chassis_data.distance_m >= 0.0f) ?
                         chassis_data.distance_m : -chassis_data.distance_m;
            remaining_distance_m = TRACK_LAP_DISTANCE_M - distance_m;
            finish_line_detected =
                (distance_m >= TRACK_FINISH_ARM_DISTANCE_M) &&
                (track.data.black_count >= TRACK_FINISH_LINE_MIN_BLACK_COUNT);

            if (finish_line_detected ||
                (remaining_distance_m <= TRACK_POSITION_TOLERANCE_M) ||
                (track_elapsed_ms >= TRACK_MAX_RUN_TIME_MS))
            {
                trackStop(now_tick);
            }
            else
            {
                forward_speed =
                    trackCalculateForwardSpeed(remaining_distance_m);
                chassis.set_velocity(&chassis, forward_speed, -turn_speed);
            }
        }
        else
        {
            chassis.set_velocity(&chassis, 0.0f, 0.0f);
        }
        osDelay(20);
    }
}

static void trackStart(uint32_t start_tick)
{
    chassis.set_velocity(&chassis, 0.0f, 0.0f);
    chassis.reset_odometry(&chassis);
    track.pid.reset(&track.pid);
    track_start_tick = start_tick;
    track_elapsed_ms = 0U;
    track_total_time_ms = 0U;
    track_running = true;
    led_red.on(&led_red);
}

static void trackStop(uint32_t stop_tick)
{
    chassis.set_velocity(&chassis, 0.0f, 0.0f);
    track.pid.reset(&track.pid);
    track_elapsed_ms = stop_tick - track_start_tick;
    track_total_time_ms = track_elapsed_ms;
    track_running = false;
    led_red.off(&led_red);
}

static float trackCalculateForwardSpeed(float remaining_distance_m)
{
    float forward_speed = TRACK_POSITION_KP * remaining_distance_m;

    if (forward_speed > track.init_config.base_speed)
        forward_speed = track.init_config.base_speed;
    if (forward_speed < 0.0f)
        forward_speed = 0.0f;

    return forward_speed;
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

    if(key == &key2)
    {
        if (event == KEY_EVENT_PRESS)
        {
            if (mode == ROBOT_MODE_TRACK)
            {
                if (!track_running && !track_start_requested)
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
        else
        {
            mode = ((mode < 2U) || (mode >= 6U)) ?
                   2U : (uint8_t)(mode + 1U);
            if (mode != ROBOT_MODE_TRACK)
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
        .base_speed = 0.4f,
        .max_turn_speed = 0.8f,
        .pid_config = {
            .kp = 0.15f,
            .ki = 0.0f,
            .kd = 0.0f,
            .enable_output_limit = true,
            .output_min = -0.8f,
            .output_max = 0.8f,
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
