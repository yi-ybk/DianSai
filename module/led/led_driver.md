# LED 模块使用说明

## 1. 模块功能

`led_driver` 基于 `bsp_gpio` 提供逻辑点亮、熄灭、翻转、状态写入和状态读取，支持高电平点亮与低电平点亮的 LED。

## 2. 硬件配置

在 CubeMX 中将 LED 引脚配置为 GPIO Output，并根据硬件确定初始电平。模块初始化配置中的 `active_state` 表示“点亮 LED”对应的实际引脚电平：

| 电路 | `active_state` |
| --- | --- |
| GPIO 输出高电平时点亮 | `GPIO_PIN_SET` |
| GPIO 下拉灌电流时点亮 | `GPIO_PIN_RESET` |

## 3. 初始化与控制

```c
#include "led_driver.h"

static Led_t green_led = { LED_OBJECT_DEFAULT };

static const LedInitConfig_t green_led_config = {
    .GPIOx = GPIOB,
    .GPIO_Pin = GPIO_PIN_2,
    .active_state = GPIO_PIN_SET,
    .init_state = LED_STATE_OFF,
    .id = "green",
};

bool GreenLedInit(void)
{
    return green_led.init(&green_led, &green_led_config);
}

void GreenLedTest(void)
{
    green_led.on(&green_led);
    green_led.off(&green_led);
    green_led.toggle(&green_led);
    green_led.write(&green_led, LED_STATE_ON);
}
```

也可以对清零对象直接调用 `LedInit()`。清零对象初始化前不能调用 `led.init()`，因为函数指针为 `NULL`。

## 4. 状态读取

```c
LedState_t state = green_led.read(&green_led);
LedData_t data;

green_led.get_data(&green_led, &data);
```

`state` 是逻辑亮灭状态，不是原始 GPIO 电平。`toggle_count` 只统计通过 LED 对象执行的翻转；如果其他代码直接调用 `HAL_GPIO_WritePin()` 或 `HAL_GPIO_TogglePin()`，硬件状态虽然能被重新读取，但翻转计数不会同步。

## 5. 与按键模块配合

```c
static void KeyCallback(Key_t *key, KeyEvent_t event, void *context)
{
    Led_t *led = (Led_t *)context;

    if ((event == KEY_EVENT_PRESS) && (led != NULL))
        led->toggle(led);
}
```

按键回调在中断上下文执行。LED GPIO 操作较短，可以直接翻转；复杂灯效和延时应交给任务处理。

## 6. 注意事项

- `GPIORegister()` 当前使用动态内存，建议在 FreeRTOS 调度器启动后的初始化任务中调用；
- 不要同时用 HAL GPIO API 和 LED 对象控制同一引脚，否则对象统计和应用逻辑容易混乱；
- 初始化失败时检查 GPIO 端口、引脚、有效电平和 BSP GPIO 实例分配；
- `id` 仅作为用户标识保存，模块不会复制、释放或解释该指针。
