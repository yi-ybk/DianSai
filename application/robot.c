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
#include "slaver.h"

/************ 函数声明 **************/
static void testTask(void *argument);
static void oledTask(void *argument);
static void imuParseTask(void *argument);
static void trackTask(void *argument);
static void blanceTask(void *argument);
static void slaverFloatTask(void *argument);
static bool slaverInit(void);
static void key1ProcessEvents(void);
static void oledDrawDefaultPage(void);
static void oledDrawBallPage(void);

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

osThreadId_t blanceTaskHandle;
const osThreadAttr_t blanceTask_attributes = {
  .name = "blanceTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static StaticTask_t slaverFloatTaskControlBlock;
static StackType_t slaverFloatTaskStack[256];
osThreadId_t slaverFloatTaskHandle;
const osThreadAttr_t slaverFloatTask_attributes = {
  .name = "slaverFloatTask",
  .cb_mem = &slaverFloatTaskControlBlock,
  .cb_size = sizeof(slaverFloatTaskControlBlock),
  .stack_mem = slaverFloatTaskStack,
  .stack_size = sizeof(slaverFloatTaskStack),
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

static UART_HandleTypeDef huart_slaver = {
    .Instance = UART_SLAVER_INST,
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

Slaver_t slaver_float = { SLAVER_OBJECT_DEFAULT };
SlaverSimpleFloatProtocol_t slaver_float_protocol;
/*******************************/

/**************全局参数***************/
float ball_target = 0.0f;
float ball_real   = 10.0f;

float target_5cm  = 5.0f;
float target_f5cm = -5.0f;

volatile uint8_t mode = 2U;
volatile uint32_t oled_refresh_error_count = 0U;
volatile uint32_t oled_recovery_count = 0U;

static volatile bool ball_menu_active = false;
static volatile bool key1_irq_pressed = false;
static volatile bool key1_oled_recovery_pending = false;
static volatile uint32_t key1_irq_press_tick = 0U;
static volatile uint32_t key1_irq_short_count = 0U;
static volatile uint32_t key1_irq_long_count = 0U;
static volatile uint32_t key1_irq_edge_tick = 0U;
static uint32_t key1_handled_short_count = 0U;
static uint32_t key1_handled_long_count = 0U;
static uint32_t key1_last_action_tick = 0U;
/************************************/

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
    testTaskHandle        = osThreadNew(testTask, NULL, &testTask_attributes);
    oledTaskHandle        = osThreadNew(oledTask, NULL, &oledTask_attributes);
    imu0TaskHandle        = osThreadNew(imuParseTask, &imu0, &imu0Task_attributes);
    trackTaskHandle       = osThreadNew(trackTask, NULL, &trackTask_attributes);
    blanceTaskHandle      = osThreadNew(blanceTask, NULL, &blanceTask_attributes);
    slaverFloatTaskHandle = osThreadNew(slaverFloatTask, NULL,
                                        &slaverFloatTask_attributes);
    configASSERT(testTaskHandle != NULL);
    configASSERT(oledTaskHandle != NULL);
    configASSERT(imu0TaskHandle != NULL);
    configASSERT(trackTaskHandle != NULL);
    configASSERT(blanceTaskHandle != NULL);
    configASSERT(slaverFloatTaskHandle != NULL);
}

static void testTask(void *argument)
{
    (void)argument;

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
    uint32_t last_refresh_tick;

    (void)argument;
    last_refresh_tick = HAL_GetTick() - 200U;

    while (1)
    {
        key1ProcessEvents();

        if ((HAL_GetTick() - last_refresh_tick) >= 200U)
        {
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

            last_refresh_tick = HAL_GetTick();
        }

        osDelay(10);
    }
}

static void oledDrawDefaultPage(void)
{
    ImuData_t imu_data;
    uint32_t elapsed_ms;
    uint32_t total_seconds;
    uint32_t hours;
    uint32_t minutes;
    uint32_t seconds;
    uint32_t tenths;
    char time_text[] = "TIME 00:00:00.0";

    imu0.get_data(&imu0, &imu_data);
    oled.draw_string(&oled, 0, 0, "mode:", OLED_COLOR_WHITE);
    oled.draw_int(&oled, 5, 0, mode, OLED_COLOR_WHITE);

    elapsed_ms = HAL_GetTick();
    total_seconds = elapsed_ms / 1000U;
    hours = (total_seconds / 3600U) % 100U;
    minutes = (total_seconds / 60U) % 60U;
    seconds = total_seconds % 60U;
    tenths = (elapsed_ms / 100U) % 10U;

    time_text[5]  = (char)('0' + (hours / 10U));
    time_text[6]  = (char)('0' + (hours % 10U));
    time_text[8]  = (char)('0' + (minutes / 10U));
    time_text[9]  = (char)('0' + (minutes % 10U));
    time_text[11] = (char)('0' + (seconds / 10U));
    time_text[12] = (char)('0' + (seconds % 10U));
    time_text[14] = (char)('0' + tenths);
    oled.draw_string(&oled, 0, 1, time_text, OLED_COLOR_WHITE);
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

static void slaverFloatTask(void *argument)
{
    (void)argument;

    if (!slaverInit())
    {
        configASSERT(0);
        vTaskDelete(NULL);
        return;
    }

    slaver_float.task(&slaver_float);
}

static bool slaverInit(void)
{
    const SlaverSimpleFloatProtocolConfig_t protocol_config = {
        .frame_header = 0xA5U,
        .frame_tail = 0x5AU,
        .frame_length = 7U,
    };
    const SlaverInitConfig_t config = {
        .uart_handle = &huart_slaver,
        .dma_config = {
            .dma = DMA,
            .rx_channel = DMA_CH_SLAVER_RX_CHAN_ID,
            .tx_channel = USART_DMA_CHANNEL_INVALID,
        },
        .uart_irqn = UART_SLAVER_INST_INT_IRQN,
        .uart_irq_priority = 2U,
        .dma_rx_buffer_size = 64U,
        .header = { 0xA5U },
        .header_length = 1U,
        .frame_length_callback = SlaverSimpleFloatFrameLength,
        .frame_validate_callback = SlaverSimpleFloatFrameValidate,
        .frame_callback = SlaverSimpleFloatFrameReceived,
        .protocol_context = &slaver_float_protocol,
        .tx_timeout_ms = 100U,
    };

    if (!SlaverSimpleFloatProtocolInit(&slaver_float_protocol,
                                       &protocol_config))
    {
        return false;
    }

    return slaver_float.init(&slaver_float, &config);
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

static void blanceTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        SlaverSimpleFloatData_t latest;
        SlaverSimpleFloatGetLatest(&slaver_float_protocol, &latest);
        ball_real = latest.value;
        osDelay(5);
    }
}

void UART_IMU0_INST_IRQHandler(void)
{
    USARTIRQHandler(&huart_imu0);
}

void UART_SLAVER_INST_IRQHandler(void)
{
    USARTIRQHandler(&huart_slaver);
}

void MCAN_GIMBAL_INST_IRQHandler(void)
{
    CANIRQHandler(MCAN_GIMBAL_INST, DL_MCAN_RX_FIFO_NUM_0);
}

void keyEventCallback(Key_t *key, KeyEvent_t event, void *context)
{
    (void)context;

    if (key == &key1)
    {
        if (event == KEY_EVENT_PRESS)
        {
            key1_irq_press_tick = key->data.last_event_tick;
            key1_irq_pressed = true;
        }
        else if ((event == KEY_EVENT_RELEASE) && key1_irq_pressed)
        {
            if ((key->data.last_event_tick - key1_irq_press_tick) >= 2000U)
                key1_irq_long_count++;
            else
                key1_irq_short_count++;

            key1_irq_pressed = false;
        }
    }
    else if (key == &key2)
    {
        if (event == KEY_EVENT_PRESS)
        {
            led_red.toggle(&led_red);
        }
    }
}

static void key1ProcessEvents(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t short_count;
    uint32_t long_count;
    uint32_t edge_tick;
    bool recover_oled = false;
    bool toggle_menu;
    bool short_press;

    taskENTER_CRITICAL();
    short_count = key1_irq_short_count;
    long_count = key1_irq_long_count;
    edge_tick = key1_irq_edge_tick;
    if (key1_oled_recovery_pending &&
        ((now_tick - edge_tick) >= 100U))
    {
        key1_oled_recovery_pending = false;
        recover_oled = true;
    }
    taskEXIT_CRITICAL();

    short_press = (short_count != key1_handled_short_count);
    toggle_menu = (long_count != key1_handled_long_count);
    key1_handled_short_count = short_count;
    key1_handled_long_count = long_count;

    /*
     * PA18 的按键边沿可能在相邻 PA17（OLED SDA）上产生毛刺。等待机械
     * 抖动结束后恢复 SSD1306 控制器状态，避免一次毛刺造成永久黑屏。
     */
    if (recover_oled)
    {
        if (oled.recover(&oled))
            oled_recovery_count++;
        else
            oled_refresh_error_count++;
    }

    /*
     * 机械按键可能在一次完整按压过程中出现超过消抖时间的二次跳变。
     * 对已经确认的动作增加锁定窗口，避免一次按键让 mode 连跳两次。
     */
    if ((toggle_menu || short_press) &&
        ((now_tick - key1_last_action_tick) < 300U))
    {
        toggle_menu = false;
        short_press = false;
    }

    if (toggle_menu)
    {
        key1_last_action_tick = now_tick;
        ball_menu_active = !ball_menu_active;
    }
    else if (short_press)
    {
        key1_last_action_tick = now_tick;
        if (ball_menu_active)
        {
            ball_target = ball_real;
        }
        else
        {
            mode = ((mode < 2U) || (mode >= 6U)) ?
                   2U : (uint8_t)(mode + 1U);
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
        .debounce_ms    = 80U,
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
    key1_last_action_tick = HAL_GetTick() - 300U;

    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_KEY_KEY_A18_PIN);
    DL_GPIO_enableInterrupt(GPIOA, GPIO_KEY_KEY_A18_PIN);

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
                    key1_irq_edge_tick = HAL_GetTickFromISR();
                    key1_oled_recovery_pending = true;
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
