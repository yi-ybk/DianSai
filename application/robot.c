#include "robot.h"
#include "cmsis_os.h"
#include "ti_msp_dl_config.h"

#include "led_driver.h"
#include "oled_driver.h"
#include "key_driver.h"

/************ 函数声明 **************/
static void testTask(void *argument);
static void oledTask(void *argument);

static void ledInit(void);
static void oledInit(void);
static void keyInit(void);
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
/*******************************/

/*********** 对象实例 ***********/
Led_t led_green = { LED_OBJECT_DEFAULT };
Led_t led_red   = { LED_OBJECT_DEFAULT };
Led_t led_blue  = { LED_OBJECT_DEFAULT };

Oled_t oled = { OLED_OBJECT_DEFAULT };

Key_t key1 = { KEY_OBJECT_DEFAULT };
/*******************************/

void robotInit(void)
{
    /* 初始化机器人相关的硬件和软件组件 */
    ledInit();
    oledInit();
    keyInit();
    testTaskHandle = osThreadNew(testTask, NULL, &testTask_attributes);
    oledTaskHandle = osThreadNew(oledTask, NULL, &oledTask_attributes);
    configASSERT(testTaskHandle != NULL);
    configASSERT(oledTaskHandle != NULL);
}

static void testTask(void *argument)
{
    (void)argument;

    while (1)
    {
        led_green.toggle(&led_green);
        led_red.toggle(&led_red);
        osDelay(500);
    }
}

void oledTask(void *argument)
{
    (void)argument;

    while (1)
    {
        oled.draw_string(&oled, 0, 0, "Hello, OLED!", OLED_COLOR_WHITE);
        oled.refresh(&oled);
        osDelay(1000);
    }
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
        .debounce_ms    = 50,
        .event_callback = keyEventCallback,
        .event_context  = NULL,
    };
    key1.init(&key1, &key1_config);
}
