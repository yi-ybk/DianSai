# 软件正交编码器使用说明

软件编码器已经集成到 `Encoder_t`，可以与现有 `Motor_t`、`Wheel_t` 直接配合使用。

## SysConfig 配置

为编码器 A、B 两相各创建一个 GPIO 输入：

- 两个引脚都启用中断；
- 中断极性选择 `RISE_FALL`；
- 根据编码器输出类型选择上拉，无外部上拉时可启用内部上拉；
- 两个引脚所在端口的 GROUP 中断必须在 NVIC 中使能。

## 初始化示例

```c
Encoder_t encoder_right = { ENCODER_OBJECT_DEFAULT };

static const EncoderInitConfig_t encoder_right_config = {
    .mode = ENCODER_MODE_SOFTWARE_GPIO,
    .phase_a = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_6,
    },
    .phase_b = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_7,
    },
    .reversed = false,
    .counts_per_rev = 1560.0f,
    .auto_start = true,
    .speed_window_samples = 4U,
};

bool EncoderRightInit(void)
{
    return encoder_right.init(&encoder_right, &encoder_right_config);
}
```

示例中的 PB6/PB7 仅用于说明，实际代码必须替换为 SysConfig 分配的空闲引脚。

## 中断接入

GPIO GROUP 中断服务函数识别出 A 相或 B 相后，必须调用：

```c
GPIOIRQHandler(encoder_port, encoder_phase_pin);
```

模块会在 GPIO 回调中同时读取 A、B 两相，并通过四状态查表完成正交解码。每个有效边沿累计一次，因此 `counts_per_rev` 应填写四倍频后的实际每圈计数。

任务中仍需周期调用：

```c
encoder_right.update(&encoder_right, dt_s);
```

`data.transition_error_count` 记录 `00 -> 11`、`01 -> 10` 等非法跳变。该值持续增加通常表示中断丢失、编码器信号过快、接线干扰或输入边沿不稳定。

软件解码每个有效边沿都会进入一次 GPIO 中断，不适合脉冲频率很高的编码器。高速侧优先使用 TIMG8 硬件 QEI，低速侧使用软件编码器。
