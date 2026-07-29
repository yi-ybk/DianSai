# 蜂鸣器模块

## 功能

`buzzer_driver` 支持两类蜂鸣器：

- 有源蜂鸣器：通过普通 GPIO 控制开关，可配置高电平或低电平鸣响。
- 无源蜂鸣器：通过 PWM 控制，可动态设置频率和占空比。

模块不会阻塞任务。需要定时鸣叫时，应由 FreeRTOS 任务或软件定时器负责延时后调用 `off()`。

## 有源蜂鸣器示例

```c
#include "buzzer_driver.h"

Buzzer_t buzzer = { BUZZER_OBJECT_DEFAULT };

static const BuzzerInitConfig_t buzzer_config = {
    .type = BUZZER_TYPE_ACTIVE_GPIO,
    .gpio = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_0,
        .active_state = GPIO_PIN_SET,
    },
    .init_state = BUZZER_STATE_OFF,
    .id = "active buzzer",
};

void BuzzerDeviceInit(void)
{
    buzzer.init(&buzzer, &buzzer_config);
}
```

```c
buzzer.on(&buzzer);
osDelay(100U);
buzzer.off(&buzzer);
```

有源蜂鸣器引脚需要在 CubeMX 中配置为 GPIO 输出。

## 无源蜂鸣器示例

```c
Buzzer_t buzzer = { BUZZER_OBJECT_DEFAULT };

static const BuzzerInitConfig_t buzzer_config = {
    .type = BUZZER_TYPE_PASSIVE_PWM,
    .pwm = {
        .htim = &htim2,
        .channel = TIM_CHANNEL_1,
        .frequency_hz = 2000.0f,
        .duty_ratio = 0.5f,
    },
    .init_state = BUZZER_STATE_OFF,
    .id = "passive buzzer",
};
```

```c
buzzer.set_tone(&buzzer, 1000.0f, 0.5f);
buzzer.on(&buzzer);

/* 运行中切换音调 */
buzzer.set_tone(&buzzer, 2000.0f, 0.5f);

buzzer.off(&buzzer);
```

无源蜂鸣器对应的定时器通道和 GPIO 需要在 CubeMX 中配置为 PWM 输出。不要与电机等其他设备共用同一个定时器通道。
