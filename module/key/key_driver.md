# Key 按键模块使用说明

## 1. 模块功能

`key_driver` 基于 `bsp_gpio` 封装通用按键对象，支持：

- 高电平或低电平有效按键；
- GPIO 外部中断触发；
- 按下和释放事件；
- 中断消抖；
- 自定义事件回调和用户上下文；
- 查询当前状态、最近事件和事件统计数据。

当前模块不包含长按、双击和连击识别。这些功能应在应用层根据按键事件和时间戳实现。

## 2. 依赖文件

工程需要包含以下文件：

```text
module/key/key_driver.c
module/key/key_driver.h
bsp/gpio/bsp_gpio.c
bsp/gpio/bsp_gpio.h
```

并添加头文件搜索路径：

```text
-Imodule/key
-Ibsp/gpio
```

## 3. CubeMX 配置

以低电平按下的按键为例：

1. 将按键引脚配置为 `GPIO_EXTI`。
2. 如果按键释放时引脚为高电平，配置上拉；如果硬件已有外部上拉，可配置为无上下拉。
3. 若需要同时识别按下和释放，将触发边沿配置为上升沿和下降沿。
4. 在 NVIC 中使能该 EXTI 中断。
5. 确认生成的 `xxx_IRQHandler()` 中调用了 `HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_x)`。

`KeyInitConfig_t.exti_mode` 必须与 CubeMX 配置保持一致：

| CubeMX 触发方式 | `exti_mode` |
| --- | --- |
| 上升沿 | `GPIO_EXTI_MODE_RISING` |
| 下降沿 | `GPIO_EXTI_MODE_FALLING` |
| 上升沿和下降沿 | `GPIO_EXTI_MODE_RISING_FALLING` |

模块在每次 EXTI 中断中读取当前引脚电平，并根据 `active_state` 生成事件。边沿与事件的对应关系如下：

| `active_state` | 上升沿事件 | 下降沿事件 |
| --- | --- | --- |
| `GPIO_PIN_SET`，高电平按下 | `KEY_EVENT_PRESS` | `KEY_EVENT_RELEASE` |
| `GPIO_PIN_RESET`，低电平按下 | `KEY_EVENT_RELEASE` | `KEY_EVENT_PRESS` |

因此，单边沿配置只会收到该边沿对应的一类事件。例如高电平按下的 PA0 使用下拉和 `GPIO_EXTI_MODE_RISING` 时，按下只产生一次 `KEY_EVENT_PRESS`；松开是下降沿，不会产生 `KEY_EVENT_RELEASE`。需要识别完整的按下和松开动作时，CubeMX 和 `exti_mode` 都必须配置为双边沿。

机械抖动或引脚电平在 ISR 读取前发生变化时，事件可能与触发边沿不一致。`debounce_ms` 可以过滤短时间内的重复边沿，但不会延迟后再次确认电平。

## 4. 基本使用示例

```c
#include "key_driver.h"
#include "led_driver.h"

static Key_t user_key = { KEY_OBJECT_DEFAULT };
static Led_t green_led = { LED_OBJECT_DEFAULT };

/* 先完成 green_led 的 LedInit()，再初始化 user_key。 */

static void UserKeyEventCallback(Key_t *key,
                                 KeyEvent_t event,
                                 void *context)
{
    Led_t *led = (Led_t *)context;

    if ((key == &user_key) &&
        (event == KEY_EVENT_PRESS) &&
        (led != NULL))
    {
        LedToggle(led);
    }
}

bool UserKeyInit(void)
{
    const KeyInitConfig_t config = {
        .GPIOx          = GPIOC,
        .GPIO_Pin       = GPIO_PIN_13,
        .active_state   = GPIO_PIN_RESET,
        .exti_mode      = GPIO_EXTI_MODE_RISING_FALLING,
        .debounce_ms    = 20U,
        .event_callback = UserKeyEventCallback,
        .event_context  = &green_led,
    };

    return user_key.init(&user_key, &config);
}
```

也可以不使用对象函数指针，直接调用：

```c
static Key_t user_key = {0};

if (!KeyInit(&user_key, &config))
{
    /* 初始化失败处理 */
}
```

不能对清零后的对象直接调用 `user_key.init()`，因为此时函数指针为 `NULL`。

回调函数的返回类型必须为 `void`，与 `KeyEventCallback_t` 保持一致：

```c
typedef void (*KeyEventCallback_t)(Key_t *key,
                                   KeyEvent_t event,
                                   void *context);
```

不要将回调定义为 `bool` 或其他返回类型。

### 回调参数说明

`key` 是触发事件的按键对象。多个按键共用同一个回调时，可通过 `key == &user_key` 区分事件来源，也可调用 `KeyRead()` 或 `KeyGetData()` 获取该按键状态。

`event` 是本次事件类型，取值为 `KEY_EVENT_PRESS` 或 `KEY_EVENT_RELEASE`。若按下和松开都执行翻转，完整按键动作会翻转两次，最终看起来没有变化；通常应只处理 `KEY_EVENT_PRESS`。

`context` 是初始化时 `event_context` 传入的用户上下文指针。Key 模块不会解释或释放它，可用于传递 LED、控制器或应用状态对象。示例中传入 `&green_led`，因此同一个回调可以复用于控制不同的 LED 对象。

## 5. 读取按键状态

读取当前逻辑状态：

```c
if (user_key.read(&user_key) == KEY_STATE_PRESSED)
{
    /* 按键当前处于按下状态 */
}
```

`KeyRead()` 会读取 GPIO 当前电平，并根据 `active_state` 转换为 `KEY_STATE_PRESSED` 或 `KEY_STATE_RELEASED`。

## 6. 读取和清除最近事件

应用任务可以轮询最近一次中断事件：

```c
KeyEvent_t event = user_key.get_last_event(&user_key);

if (event != KEY_EVENT_NONE)
{
    if (event == KEY_EVENT_PRESS)
    {
        /* 处理按下事件 */
    }

    user_key.clear_event(&user_key);
}
```

`last_event` 只保存最近一次事件，不是事件队列。如果应用层处理不及时，连续事件可能覆盖前一次事件。需要保证每次事件都不丢失时，应在回调中向 FreeRTOS 队列或任务通知发送事件。

## 7. 获取运行数据

```c
KeyData_t data;

user_key.get_data(&user_key, &data);
```

`KeyData_t` 中包含：

| 字段 | 说明 |
| --- | --- |
| `state` | 当前逻辑状态 |
| `last_event` | 最近一次按键事件 |
| `pin_state` | 当前 GPIO 实际电平 |
| `event_count` | 有效事件总数 |
| `press_count` | 按下事件次数 |
| `release_count` | 释放事件次数 |
| `last_event_tick` | 最近一次有效事件的 HAL tick |

`get_data()` 会在短临界区内复制数据，可用于任务中读取中断更新的数据快照。

## 8. 动态更换回调

```c
user_key.set_callback(&user_key, NewKeyCallback, new_context);
```

传入 `NULL` 可以停止调用用户回调，但按键状态和事件统计仍会更新：

```c
user_key.set_callback(&user_key, NULL, NULL);
```

## 9. 消抖说明

`debounce_ms` 表示两次有效外部中断事件之间允许的最小时间：

```c
.debounce_ms = 20U,
```

在消抖时间内发生的新边沿会被忽略。设置为 `0U` 可关闭软件消抖。

当前消抖基于 `HAL_GetTick()`，因此必须保证 HAL tick 正常运行。消抖是边沿时间过滤，不会延迟后再次确认引脚电平；机械抖动严重时，可在应用层增加定时复检。

## 10. 中断回调限制

`event_callback` 在 GPIO 外部中断上下文中执行，应遵守以下要求：

- 回调应尽量短小；
- 不要调用 `osDelay()`、`HAL_Delay()` 或其他阻塞函数；
- 不要在回调中执行格式化打印、动态内存分配或复杂计算；
- 使用 FreeRTOS API 时，只能调用对应的 `FromISR` 接口；
- 复杂逻辑应通过任务通知、信号量或队列交给任务处理。

## 11. 初始化时机

当前 `GPIORegister()` 使用标准库 `malloc()` 创建 GPIO 实例。本工程启用了 FreeRTOS C 运行库锁时，不应在调度器启动前调用 `KeyInit()`，否则可能影响中断恢复和 HAL tick。

推荐在 FreeRTOS 调度器启动后的任务中初始化按键对象，或者后续将 `bsp_gpio` 改为静态对象池。

## 12. 常见问题

### 初始化返回 `false`

检查以下项目：

- `Key_t` 和 `KeyInitConfig_t` 指针是否有效；
- `GPIOx` 是否为有效 GPIO 端口；
- `GPIO_Pin` 是否为非零的 `GPIO_PIN_x`；
- `active_state` 是否为 `GPIO_PIN_SET` 或 `GPIO_PIN_RESET`；
- `exti_mode` 是否为上升沿、下降沿或双边沿，不能使用 `GPIO_EXTI_MODE_NONE`；
- `GPIORegister()` 是否成功分配实例。

### 没有触发回调

检查 CubeMX 是否已配置 EXTI 和 NVIC，并确认 `exti_mode` 与实际触发边沿一致。还应确认工程中保留了 `bsp_gpio.c` 提供的 `HAL_GPIO_EXTI_Callback()`，避免被其他文件重复定义或覆盖。

### 只能收到按下或只能收到释放

如果需要同时识别按下和释放，GPIO 必须配置双边沿触发，并使用：

```c
.exti_mode = GPIO_EXTI_MODE_RISING_FALLING,
```

高电平按下时，上升沿为 `KEY_EVENT_PRESS`、下降沿为 `KEY_EVENT_RELEASE`；低电平按下时正好相反。还要同步修改 CubeMX 的 EXTI 触发方式。

### 多个 GPIO 使用相同 Pin 编号

STM32 的同一条 EXTI 线一次只能映射到一个 GPIO 端口。例如 `PA0` 和 `PB0` 不能同时作为独立的 `EXTI0` 输入使用。
