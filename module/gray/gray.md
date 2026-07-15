# Gray 数字灰度传感器模块

## 功能

该模块基于 `bsp_gpio` 读取数字灰度传感器阵列，支持：

- 配置 `1~16` 路灰度输入；
- 配置高电平或低电平代表黑线；
- 单通道读取黑色或白色；
- 获取整组黑线位图和白色位图。

模块只负责读取数字电平。GPIO 输入模式和上下拉方式仍需在 CubeMX 中配置。

## 初始化示例

```c
#include "gray.h"

Gray_t gray = { GRAY_OBJECT_DEFAULT };

static const GrayChannelConfig_t gray_channels[] = {
    { .GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_0 },
    { .GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_1 },
    { .GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_2 },
    { .GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_3 },
};

static const GrayInitConfig_t gray_config = {
    .channels = gray_channels,
    .channel_count = sizeof(gray_channels) / sizeof(gray_channels[0]),
    .black_state = GPIO_PIN_RESET,
};

void GraySensorInit(void)
{
    gray.init(&gray, &gray_config);
}
```

`black_state = GPIO_PIN_RESET` 表示低电平为黑线、高电平为白色；设置为 `GPIO_PIN_SET` 时含义相反。

## 数据读取

```c
GrayData_t data;

gray.update(&gray);
gray.get_data(&gray, &data);

if (gray.read_channel(&gray, 0U) == GRAY_COLOR_BLACK)
{
    /* 第0路检测到黑线 */
}
```

`black_mask` 中 bit0 对应配置数组第0路，bit1 对应第1路，以此类推。bit为1表示该路检测到黑线；`white_mask` 的位含义相同，但bit为1表示检测到白色。
