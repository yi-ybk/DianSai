# Encoder 编码器模块使用说明

## 1. 模块功能

`encoder` 基于 `bsp_tim` 的定时器编码器模式，提供累计计数、单周期增量、方向、`count/s` 和 `rev/s`。模块处理定时器计数回绕，并支持通过 `reversed` 统一机械安装方向。

## 2. 依赖与硬件配置

需要包含：

```text
module/encoder/encoder.c
bsp/tim/bsp_tim.c
```

在 CubeMX 中将定时器配置为 Encoder Mode，配置 CH1/CH2 引脚、计数周期和输入滤波。调用 `EncoderInit()` 前先执行对应的 `MX_TIMx_Init()`。

`counts_per_rev` 应填写定时器实际累计的每圈计数，而不是只看编码器标称线数。它是否包含四倍频和减速比取决于定时器配置和编码器安装位置。

## 3. 初始化示例

```c
#include "encoder.h"
#include "tim.h"

static Encoder_t left_encoder = { ENCODER_OBJECT_DEFAULT };

static const EncoderInitConfig_t left_encoder_config = {
    .htim = &htim2,
    .channel = TIM_CHANNEL_ALL,
    .reversed = false,
    .counts_per_rev = 1560.0f,
    .auto_start = true,
};

bool LeftEncoderInit(void)
{
    return left_encoder.init(&left_encoder, &left_encoder_config);
}
```

也可使用清零对象并直接调用普通函数：

```c
static Encoder_t encoder = {0};

if (!EncoderInit(&encoder, &config))
{
    /* 初始化失败处理 */
}
```

清零对象的函数指针为空，初始化前不能调用 `encoder.init()`。

## 4. 周期更新和读取

```c
void EncoderTask(void *argument)
{
    const float dt_s = 0.01f;
    EncoderData_t data;

    for (;;)
    {
        left_encoder.update(&left_encoder, dt_s);
        left_encoder.get_data(&left_encoder, &data);

        /* data.count、data.speed_rps、data.direction */
        osDelay(10U);
    }
}
```

`dt_s` 必须是两次 `update()` 之间的实际秒数。周期不稳定时应由 DWT 或系统 tick 计算真实周期，否则速度会有比例误差。

`EncoderData_t` 主要字段：

| 字段 | 含义 |
| --- | --- |
| `count` | 软件累计计数，可跨定时器回绕 |
| `delta` | 本次更新相对上次的计数增量 |
| `raw_count` | 当前定时器原始计数 |
| `direction` | 根据 `delta` 判断的方向 |
| `speed_cps` | 每秒计数 |
| `speed_rps` | 编码器轴每秒转数 |

## 5. 启停与复位

```c
left_encoder.stop(&left_encoder);
left_encoder.start(&left_encoder);
left_encoder.reset(&left_encoder);
```

`stop()` 停止底层定时器并清零瞬时速度，但保留累计 `count`。`reset()` 会将硬件计数器和累计里程数据清零。

## 6. 与 Motor/Wheel 的关系

编码器对象应先于电机初始化：

```text
EncoderInit
    -> MotorInit（绑定 Encoder_t*）
        -> WheelInit（绑定 Motor_t*）
```

`MotorUpdate()` 会调用绑定编码器的 `EncoderUpdate()`，因此编码器被 Motor/Wheel 管理后不要在其他任务中重复更新，否则速度周期和增量会被破坏。原始计数由 Encoder 模块读取；上层通常通过 Wheel 获取线速度和里程。

## 7. 常见问题

- 方向相反：切换 `.reversed`，不要同时在 Encoder、Motor 和 Wheel 多层重复反向。
- 速度为零：确认已启动、任务持续调用 `update()`、`counts_per_rev > 0`。
- 速度跳变：检查定时器 ARR、输入滤波、计数回绕和 `dt_s`。
- 初始化失败：检查定时器句柄、通道是否为 `TIM_CHANNEL_ALL/1/2`，以及 `bsp_tim` 实例池是否已满。
