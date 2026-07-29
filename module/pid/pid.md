# PID 组件使用说明

## 1. 模块功能

`module/pid` 提供对象式位置 PID，支持：

- 显式传入计算周期 `dt_s`；
- 输出限幅；
- 积分输出限幅；
- 误差死区；
- 对误差或测量值微分；
- 死区内可选清除积分；
- 运行数据快照。

它与 `module/algorithm/controller` 独立。普通 Motor/Wheel 闭环优先使用本组件；需要高级积分和滤波优化时再使用 `controller`。

## 2. 初始化

```c
#include "pid.h"

static Pid_t speed_pid = { PID_OBJECT_DEFAULT };

static const PidInitConfig_t speed_pid_config = {
    .kp = 1.0f,
    .ki = 0.2f,
    .kd = 0.01f,
    .enable_output_limit = true,
    .output_min = -1.0f,
    .output_max = 1.0f,
    .enable_integral_limit = true,
    .integral_min = -0.5f,
    .integral_max = 0.5f,
    .deadband = 0.01f,
    .derivative_on_measurement = true,
    .reset_integral_on_deadband = false,
};

bool SpeedPidInit(void)
{
    return speed_pid.init(&speed_pid, &speed_pid_config);
}
```

`integral_min/max` 限制的是 `i_out`，即乘以 `ki` 后的积分输出，不是原始积分累计值。

## 3. 周期计算

```c
float output = speed_pid.calculate(&speed_pid,
                                   measured_speed,
                                   target_speed,
                                   0.01f);
```

误差定义为：

```text
error = target - feedback
```

`dt_s` 单位为秒。首次计算不产生微分项；`dt_s <= 0` 时不累加积分，也不计算微分。任务周期不稳定时应传入实际测量周期。

启用 `derivative_on_measurement` 后：

```text
derivative = -(feedback - last_feedback) / dt
```

这能减小目标值阶跃造成的微分冲击。

## 4. 死区

当 `abs(error) <= deadband` 时，误差按 0 处理，不累加积分。`reset_integral_on_deadband=true` 会同时清除历史积分；否则保留原积分输出。

死区必须大于等于 0，负值会导致初始化失败。

## 5. 动态调整

```c
speed_pid.set_param(&speed_pid, new_kp, new_ki, new_kd);
speed_pid.set_output_limit(&speed_pid, true, -0.8f, 0.8f);
speed_pid.set_integral_limit(&speed_pid, true, -0.3f, 0.3f);
speed_pid.reset(&speed_pid);
```

修改 `ki` 后，已有原始积分累计量不会自动重新标定。大幅修改参数或切换控制模式时建议调用 `reset()`。

`set_target()` 只更新运行数据中的目标缓存；下一次 `calculate()` 仍以传入的 `target` 参数为准。

## 6. 运行数据

```c
PidData_t data;

speed_pid.get_data(&speed_pid, &data);
```

可查看 `error`、`integral`、`derivative`、`p_out/i_out/d_out`、最终 `output`、周期和更新次数，用于调参和故障定位。

## 7. 调参顺序

1. 先设 `ki=0`、`kd=0`，逐步增加 `kp`；
2. 增加 `ki` 消除稳态误差，并设置积分限幅；
3. 需要抑制快速变化时再增加 `kd`；
4. 根据传感器噪声设置死区；
5. 最后验证输出饱和、正反转、停机和目标阶跃。

## 8. 限制

- 输出饱和时没有自动反积分饱和回算，只能通过积分限幅控制；
- 对象没有内部互斥，多任务不能同时更新同一个 PID；
- `get_data()` 是普通结构体复制，不应与高优先级中断并发修改同一对象；
- 参数和反馈的物理单位由应用决定，但 target、feedback、限幅和增益必须保持一致。
