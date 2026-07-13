#include "robot.h"
#include "usart.h"
#include "imu_driver.h"
#include "led_driver.h"
#include "key_driver.h"

static void KeyEventCallback(Key_t *key, KeyEvent_t event, void *context);
static void OledTask(void *argument);

osThreadId_t imu0TaskHandle;
const osThreadAttr_t imu0Task_attributes = {
  .name = "imu0Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t testTaskHandle;
const osThreadAttr_t testTask_attributes = {
  .name = "testTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

Imu_t imu0 = {                                           
        IMU_OBJECT_DEFAULT,

        .protocol_frame_len    = 10,
        .protocol_header_bytes = (const uint8_t[]){0xAA, 0x55},
        .protocol_header_len   = 2,
        .protocol_tail_bytes   = (const uint8_t[]){0x0D, 0x0A},
        .protocol_tail_len     = 2,

        .init_config = {
            .recv_buff_size = IMU_UART_DMA_RX_BUFFER_LEN,
            .usart_handle   = &huart1,
            .parser         = Imu0FrameParse,
            .parser_context = NULL,
        },
};


void robotInit(void)
{
    __disable_irq();

    // imu0TaskHandle = osThreadNew(ImuParseTask, &imu0, &imu0Task_attributes);
    oledTaskHandle = osThreadNew(OledTask, NULL, &oledTask_attributes);
    testTaskHandle = osThreadNew(TestTask, NULL, &testTask_attributes);

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
    .exti_mode      = GPIO_EXTI_MODE_RISING,
    .debounce_ms    = 50,
    .event_callback = KeyEventCallback,
    .event_context  = NULL,
};

void TestTask(void *argument)
{

    green_led.init(&green_led, &green_led_config);
    key1.init(&key1, &key1_config);

    for (;;)
    {
    }
}

void ImuParseTask(void *argument)
{
    Imu_t *imu = (Imu_t *)argument;

    if (imu == NULL)
        while (1);

    if (imu->init != NULL)
        (void)imu->init(imu, &imu->init_config);

    for (;;)
    {
        if (imu->process != NULL)
            imu->process(imu);

        osDelay(1);
    }
}

void OledTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        osDelay(1000);
    }
}

void KeyEventCallback(Key_t *key, KeyEvent_t event, void *context)
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