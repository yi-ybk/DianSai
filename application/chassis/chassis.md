# Chassis 底盘组件使用说明

## 1. 组件功能

`Chassis_t` 聚合可变数量的 `Wheel_t`，只提供底盘级行为：

- 启动和停止底盘；
- 设置前进/后退线速度；
- 设置转向角速度；
- 根据各轮最大速度等比例限速；
- 周期更新所有轮子；
- 根据轮速和轮里程反算底盘速度、距离和转角；
- 可选绑定 `Imu_t`。

Chassis 不对外包装电机 PWM、编码器计数或单轮输出控制。底层对象应在系统初始化阶段完成组装，运行阶段通过 Chassis 控制。

## 2. 初始化依赖

推荐顺序：

```text
EncoderInit
    -> MotorInit
        -> PidInit
            -> WheelInit
                -> ChassisInit
```

所有 Wheel 必须已经初始化，且数组中不能重复引用同一个 Wheel 对象。`wheels` 和 `wheel_kinematics` 数组不会被复制，它们必须使用静态或全局存储，生命周期覆盖 Chassis。

## 3. 两轮差速底盘示例

设左右轮中心距离为 `track_width_m`。正转向速度定义为逆时针，左轮转向系数为负，右轮为正：

```c
#include "chassis.h"

#define CHASSIS_TRACK_WIDTH_M 0.180f

extern Wheel_t left_wheel;
extern Wheel_t right_wheel;

static Chassis_t chassis = { CHASSIS_OBJECT_DEFAULT };

static Wheel_t *const chassis_wheels[] = {
    &left_wheel,
    &right_wheel,
};

static const ChassisWheelKinematics_t chassis_kinematics[] = {
    {
        .forward_coefficient = 1.0f,
        .turn_coefficient_m = -(CHASSIS_TRACK_WIDTH_M * 0.5f),
    },
    {
        .forward_coefficient = 1.0f,
        .turn_coefficient_m = CHASSIS_TRACK_WIDTH_M * 0.5f,
    },
};

static const ChassisInitConfig_t chassis_config = {
    .wheels = chassis_wheels,
    .wheel_kinematics = chassis_kinematics,
    .wheel_count = sizeof(chassis_wheels) / sizeof(chassis_wheels[0]),
    .auto_start = false,
};

bool AppChassisInit(void)
{
    if (!chassis.init(&chassis, &chassis_config))
        return false;

    chassis.stop(&chassis);
    return true;
}
```

每个轮子的目标线速度按下式计算：

```text
wheel_speed = forward_coefficient * forward_speed
            + turn_coefficient_m * turn_speed
```

对于四轮滑移转向底盘，可以让两个左轮使用相同负转向系数，两个右轮使用相同正转向系数。

## 4. 运动学配置要求

虽然轮子数量可自定义，但系数矩阵必须能够独立求解前进速度和转向速度。当前初始化会检查最小二乘法正规矩阵的行列式；以下情况会失败：

- `wheel_count == 0`；
- Wheel 指针为空、未初始化或重复；
- 所有轮子的前进/转向系数线性相关；
- 只有一个普通驱动轮，无法同时观测前进和转向两个自由度。

当前模型只处理“前进 + 平面转向”两个自由度，不支持麦克纳姆轮的横移速度。麦轮底盘需要扩展运动学数据和 API。

## 5. 速度控制

```c
chassis.start(&chassis);

/* 0.5 m/s 前进，不转向 */
chassis.set_velocity(&chassis, 0.5f, 0.0f);

/* 原地逆时针旋转 1 rad/s */
chassis.set_velocity(&chassis, 0.0f, 1.0f);

/* 后退并顺时针转向 */
chassis.set_velocity(&chassis, -0.3f, -0.5f);
```

也可只修改一个分量：

```c
chassis.set_forward_speed(&chassis, 0.4f);
chassis.set_turn_speed(&chassis, 0.8f);
```

这两个接口会保留另一个分量最近一次请求的目标值。

如果任一轮目标速度超过该 Wheel 的 `max_linear_speed_mps`，Chassis 会按相同比例缩小所有轮速，保持运动方向和曲率。实际缩放值保存在 `wheel_speed_scale`。

## 6. 周期控制任务

```c
static void ChassisTask(void *argument)
{
    const float dt_s = 0.01f;

    chassis.start(&chassis);

    for (;;)
    {
        chassis.update(&chassis, dt_s);
        osDelay(10U);
    }
}
```

`update()` 会依次调用所有 Wheel 的 `update()`，因此不要再由其他任务更新同一组 Wheel、Motor 或 Encoder。闭环控制的 `dt_s` 应是实际周期秒数。

## 7. 获取速度和里程

```c
ChassisData_t data;

chassis.get_data(&chassis, &data);
```

主要字段：

| 字段 | 含义 |
| --- | --- |
| `target_forward_speed_mps` | 用户请求的前进速度 |
| `target_turn_speed_radps` | 用户请求的转向速度 |
| `command_*` | 等比例限速后实际下发的底盘速度 |
| `forward_speed_mps` | 根据轮速反算的前进速度 |
| `turn_speed_radps` | 根据轮速反算的转向速度 |
| `distance_m` | 根据轮里程反算的底盘中心累计距离 |
| `turn_angle_rad` | 根据轮里程反算的累计转角 |
| `enabled` | 所有 Wheel 都处于使能状态时为 true |

反馈和里程使用所有轮子的最小二乘解。它们仍属于轮式里程计，会受到轮胎打滑、轮径误差、传动间隙和编码器误差影响。

复位：

```c
chassis.reset_odometry(&chassis);
```

该操作会复位所有 Wheel 及其绑定 Encoder 的累计计数。

## 8. 安全停止

```c
chassis.stop(&chassis);
```

停止会停止全部 Wheel，并清零底盘目标和命令速度，但保留里程数据。重新启动后应重新发送目标速度。

建议在初始化失败、任务超时、通信失联和故障处理路径中都调用停止接口。由于 Motor 初始化当前会自动使能，系统启动阶段也应确保初始输出为 0，并在所有组件准备完成前保持 Chassis 停止。

## 10. 常见问题

- 初始化失败：检查对象初始化顺序、重复 Wheel、运动学矩阵和数组生命周期。
- 前进时发生旋转：检查左右轮方向、轮径、传动比和转向系数符号。
- 正转方向相反：确认正角速度定义为逆时针，并检查左右轮系数。
- 目标速度达不到：查看 `wheel_speed_scale`、各轮最大速度和电机输出限幅。
- 里程偏差大：标定轮半径、轮距、编码器每圈计数和减速比，并评估打滑。
- 设置速度后闭环不更新：确认周期调用 `ChassisUpdate()` 且所有 Wheel 已启动。
