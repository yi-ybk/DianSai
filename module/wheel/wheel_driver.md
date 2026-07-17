# Wheel 驱动轮模块使用说明

## 1. 模块定位

`Wheel_t` 是轮级组件，组合已经存在的：

- `Motor_t`：执行减速电机输出；
- `Encoder_t`：由 Motor 绑定并提供转速与累计计数；
- `Pid_t`：可选，用于线速度闭环。

Wheel 对外使用 `m/s`、`rad/s` 和 `m`，负责传动比、轮半径、方向和里程换算。原始编码器计数仍由 Encoder 管理。

## 2. 对象初始化顺序

```text
EncoderInit
    -> MotorInit（绑定 Encoder_t）
        -> PidInit（闭环时）
            -> WheelInit（绑定 Motor_t 和 Pid_t）
```

Motor 必须是 `MOTOR_TYPE_REDUCTION` 且已经初始化。启用速度 PID 时，Pid 对象也必须先初始化。

## 3. 初始化示例

```c
#include "wheel_driver.h"

static Wheel_t left_wheel = { WHEEL_OBJECT_DEFAULT };

static const WheelInitConfig_t left_wheel_config = {
    .motor = &left_motor,
    .speed_pid = &left_speed_pid,
    .use_speed_pid = true,
    .radius_m = 0.0325f,
    .encoder_to_wheel_ratio = 30.0f,
    .max_linear_speed_mps = 1.2f,
    .output_min = -1.0f,
    .output_max = 1.0f,
    .reversed = false,
    .auto_start = false,
};

bool LeftWheelInit(void)
{
    if (!left_wheel.init(&left_wheel, &left_wheel_config))
        return false;

    left_wheel.stop(&left_wheel);
    return true;
}
```

`encoder_to_wheel_ratio` 表示编码器轴转速除以该值后得到轮子转速。编码器装在 30:1 减速箱电机轴上时通常填 `30.0f`；编码器直接装在轮轴上时填 `1.0f`。

`MotorInit()` 当前会自动使能电机，因此系统要求上电保持停止时，应让 Motor 的 `init_output=0`，并在 Wheel 初始化后主动调用 `stop()`。

## 4. 开环速度控制

当 `use_speed_pid=false` 时：

```c
left_wheel.set_linear_speed(&left_wheel, 0.5f);
```

Wheel 将目标线速度除以 `max_linear_speed_mps`，映射为归一化 Motor 输出，再由 `output_min/max` 限幅。开环模式不保证实际速度达到目标值。

## 5. 闭环速度控制

当 `use_speed_pid=true` 时，设置目标只更新目标值，必须周期调用 `update()` 才会刷新编码器并计算 PID 输出：

```c
void WheelControlTask(void *argument)
{
    const float dt_s = 0.01f;

    left_wheel.start(&left_wheel);
    left_wheel.set_linear_speed(&left_wheel, 0.5f);

    for (;;)
    {
        left_wheel.update(&left_wheel, dt_s);
        osDelay(10U);
    }
}
```

闭环模式实际依赖编码器反馈。当前配置校验不会强制 Motor 绑定 Encoder；如果没有编码器，反馈速度始终为 0，PID 会持续输出错误结果。因此启用速度 PID 时必须同时正确绑定 Encoder。

PID 输出范围应与 Motor 输入范围一致，通常设置为 `-1.0f` 到 `1.0f`。

## 6. 角速度接口

```c
left_wheel.set_angular_speed(&left_wheel, 10.0f);
```

Wheel 使用以下关系换算：

```text
linear_speed = angular_speed * radius
angular_speed = linear_speed / radius
```

该角速度是轮子绕自身轴的机械角速度，不是底盘转向角速度。

## 7. 数据与里程

```c
WheelData_t data;

left_wheel.get_data(&left_wheel, &data);
```

主要字段：

| 字段 | 单位/含义 |
| --- | --- |
| `target_linear_speed_mps` | 目标轮缘线速度，`m/s` |
| `linear_speed_mps` | 编码器估算线速度，`m/s` |
| `angular_speed_radps` | 轮轴角速度，`rad/s` |
| `wheel_speed_rps` | 轮子转速，`rev/s` |
| `distance_m` | 基于累计编码器计数的轮缘里程，`m` |
| `angle_rad` | 轮子累计转角，`rad` |

里程计算要求 Motor 已绑定 Encoder，且 Encoder 的 `counts_per_rev > 0`。`reset_odometry()` 会复位绑定编码器的累计计数，这会同时影响所有引用该 Encoder 的对象。

## 8. 方向配置

`reversed=true` 会同时反转下发给 Motor 的逻辑输出和反馈速度/里程方向。应选择一个层级统一机械方向：

- 编码器 A/B 相位相反：优先调整 Encoder 的 `reversed`；
- 整个轮子安装镜像，需要统一命令和反馈：使用 Wheel 的 `reversed`；
- TB6612 IN1/IN2 定义相反：使用 Motor TB6612 的 `reversed`。

不要在多个层级同时反向后凭现象调试，双重反向会掩盖配置错误。

## 9. 启停行为

```c
left_wheel.start(&left_wheel);
left_wheel.stop(&left_wheel);
```

`stop()` 会停止 Motor、清零目标速度并复位速度 PID，但保留里程。重新启动后应重新设置目标速度。

## 10. 常见问题

- 初始化失败：检查 Motor 类型和初始化状态、轮半径、传动比、最大速度、输出范围和 Pid 状态。
- 实际方向相反：检查 Encoder、Motor、Wheel 三层反向配置。
- 里程为零：确认编码器绑定、`counts_per_rev` 和 `update()` 周期调用。
- 闭环不动作：确认 Wheel 已启动，并持续调用 `update()`。
- 闭环振荡：核对 `dt_s`、编码器单位、轮半径、减速比和 PID 输出限幅后再调参。
