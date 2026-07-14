#include "robot.h"
#include "usart.h"
#include "imu_driver.h"
#include "mg354pdh0_driver.h"
#include "led_driver.h"
#include "key_driver.h"
#include "oled_driver.h"

static void imuParseTask(void *argument);
static void testTask(void *argument);
static void oledTask(void *argument);

static void keyEventCallback(Key_t *key, KeyEvent_t event, void *context);

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


void robotInit(void)
{
    __disable_irq();

    imu0TaskHandle = osThreadNew(imuParseTask, &imu0, &imu0Task_attributes);
    oledTaskHandle = osThreadNew(oledTask, NULL, &oledTask_attributes);
    testTaskHandle = osThreadNew(testTask, NULL, &testTask_attributes);

    __enable_irq();
}

Led_t green_led = { LED_OBJECT_DEFAULT  };
LedInitConfig_t green_led_config = {
  .GPIOx        = GPIOB,
  .GPIO_Pin     = GPIO_PIN_2,
  .active_state = GPIO_PIN_SET,
  .init_state   = LED_STATE_OFF,
  .id           = "Green LED",
};

Key_t key1 = { KEY_OBJECT_DEFAULT };
KeyInitConfig_t key1_config = {
    .GPIOx          = GPIOA,
    .GPIO_Pin       = GPIO_PIN_0,
    .active_state   = GPIO_PIN_SET,
    .exti_mode      = GPIO_EXTI_MODE_RISING_FALLING,
    .debounce_ms    = 50,
    .event_callback = keyEventCallback,
    .event_context  = NULL,
};



void testTask(void *argument)
{

    green_led.init(&green_led, &green_led_config);
    key1.init(&key1, &key1_config);

    for (;;)
    {
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

void oledTask(void *argument)
{
    (void)argument;

    Oled_t oled = { OLED_OBJECT_DEFAULT };
    OledInitConfig_t oled_config = {
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


    oled.draw_string(&oled, 0, 0,  "acc:", OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 1,  "x:"  , OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 2,  "y:"  , OLED_COLOR_WHITE);
    oled.draw_string(&oled, 0, 3,  "z:"  , OLED_COLOR_WHITE);

    oled.draw_string(&oled, 10, 0, "angle:", OLED_COLOR_WHITE);
    oled.draw_string(&oled, 10, 1, "x:"    , OLED_COLOR_WHITE);
    oled.draw_string(&oled, 10, 2, "y:"    , OLED_COLOR_WHITE);
    oled.draw_string(&oled, 10, 3, "z:"    , OLED_COLOR_WHITE);
    oled.refresh(&oled);

    for (;;)
    {
        ImuData_t imu_data;
        imu0.get_data(&imu0, &imu_data);

        oled.draw_float(&oled, 3, 1, imu_data.accel.x, 2, OLED_COLOR_WHITE);
        oled.draw_float(&oled, 3, 2, imu_data.accel.y, 2, OLED_COLOR_WHITE);
        oled.draw_float(&oled, 3, 3, imu_data.accel.z, 2, OLED_COLOR_WHITE);

        oled.draw_float(&oled, 13, 1, imu_data.angle.x, 2, OLED_COLOR_WHITE);
        oled.draw_float(&oled, 13, 2, imu_data.angle.y, 2, OLED_COLOR_WHITE);
        oled.draw_float(&oled, 13, 3, imu_data.angle.z, 2, OLED_COLOR_WHITE);

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
