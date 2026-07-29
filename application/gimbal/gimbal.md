# 二维云台组件使用说明

## 1. 组件职责

`Gimbal_t` 位于应用层，用于组合两个已经初始化的 `Zdt42_t` 电机对象：

- `yaw`：水平旋转轴
- `pitch`：俯仰轴

组件只对外提供云台级行为，不要求上层直接换算电机转速或脉冲数：

- 双轴同步角度控制
- 单独设置水平或俯仰角度
- 双轴同步角速度控制
- 机械角度限位
- 双轴停止和同步回零
- 将当前位置设置为零点
- 位置、角速度、通信和动作完成状态读取

底层闭环由张大头42电机内部完成，当前组件不额外添加PID控制器。

## 2. 初始化前提

两个 `Zdt42_t` 电机必须先完成初始化，并使用不同的电机地址：

```c
#include "ti_msp_dl_config.h"
#include "gimbal.h"
#include "zhangdatou_42.h"

Zdt42_t yaw_motor = {ZDT42_OBJECT_DEFAULT};
Zdt42_t pitch_motor = {ZDT42_OBJECT_DEFAULT};
Gimbal_t gimbal = {GIMBAL_OBJECT_DEFAULT};

static const Zdt42InitConfig_t yaw_motor_config =
    ZDT42_INIT_CONFIG_DEFAULT(MCAN_GIMBAL_INST, 1U);
static const Zdt42InitConfig_t pitch_motor_config =
    ZDT42_INIT_CONFIG_DEFAULT(MCAN_GIMBAL_INST, 2U);
```

初始化电机：

```c
if (!yaw_motor.init(&yaw_motor, &yaw_motor_config) ||
    !pitch_motor.init(&pitch_motor, &pitch_motor_config))
{
    /* 电机对象初始化失败处理 */
}
```

### 2.1 `GimbalInit` 的调用时机

`gimbal.init(&gimbal, &gimbal_config)` 的函数指针最终调用 `GimbalInit()`。应用层可以再用
`static void gimbalInit(void)` 对配置和错误处理进行包装，但这个小写名称不是云台模块提供的接口。

推荐的初始化顺序为：

1. 由 `SYSCFG_DL_init()` 完成系统时钟、PA26/PA27复用和CANFD0初始化。
2. 初始化两个 `Zdt42_t` 电机对象，使其注册CAN收发实例。
3. 调用 `gimbal.init()`，绑定两个电机、配置反馈并根据 `auto_enable` 设置使能状态。
4. 创建或启动周期调用 `gimbal.update()` 的任务。
5. 初始化成功后，才调用 `set_angle()`、`set_angular_velocity()`、`home()` 等控制接口。

在当前工程中，建议在 `robotInit()` 内先调用 `zdt42Init()`，再调用应用层的
`gimbalInit()`，并将二者放在云台任务创建之前：

```c
void robotInit(void)
{
    /* 其他模块初始化 */

    zdt42Init();
    gimbalInit();

    /* 随后创建会访问gimbal对象的任务 */
}
```

如果电机驱动器上电后需要等待一段时间才能接收CAN命令，则不要在调度器启动前立即
调用 `gimbal.init()`。可在专用云台任务开始时先 `osDelay()`，再依次初始化电机对象和
云台对象；其他任务必须等待初始化成功后才能访问云台控制接口。两种方式只能选择一种，
不要在 `robotInit()` 和任务入口中重复初始化。

初始化调用必须检查返回值。返回 `false` 表示配置无效、反馈配置命令发送失败或自动使能
失败，此时不能继续下发运动命令。对象初始化成功后再次调用 `gimbal.init()` 会直接返回
`true`，不会重新应用新的配置；若需要修改配置，应在首次初始化前完成。

## 3. 云台轴参数

`GimbalAxisConfig_t` 的关键参数：

| 参数 | 含义 |
|---|---|
| `motor` | 该轴绑定的 `Zdt42_t` 对象 |
| `pulses_per_motor_rev` | 电机轴旋转一圈对应的位置命令脉冲数 |
| `motor_to_axis_ratio` | 电机转数 / 云台输出轴转数 |
| `min_angle_deg` | 该轴允许的最小机械角度 |
| `max_angle_deg` | 该轴允许的最大机械角度 |
| `position_offset_deg` | 电机反馈位置为0时对应的云台坐标角度 |
| `reversed` | 电机正方向是否与云台轴正方向相反 |
| `home_mode` | 回零模式 |

例如电机采用16细分、每圈200整步，电机直连云台轴：

```text
pulses_per_motor_rev = 200 * 16 = 3200
motor_to_axis_ratio  = 1
```

如果电机转10圈，云台轴转1圈：

```text
motor_to_axis_ratio = 10
```

## 4. 完整初始化示例

```c
static const GimbalInitConfig_t gimbal_config = {
    .yaw = {
        .motor = &yaw_motor,
        .pulses_per_motor_rev = 3200.0f,
        .motor_to_axis_ratio = 1.0f,
        .min_angle_deg = -170.0f,
        .max_angle_deg = 170.0f,
        .position_offset_deg = 0.0f,
        .reversed = false,
        .home_mode = ZDT42_HOME_NEAREST,
    },
    .pitch = {
        .motor = &pitch_motor,
        .pulses_per_motor_rev = 3200.0f,
        .motor_to_axis_ratio = 1.0f,
        .min_angle_deg = -45.0f,
        .max_angle_deg = 90.0f,
        .position_offset_deg = 0.0f,
        .reversed = true,
        .home_mode = ZDT42_HOME_NEAREST,
    },
    .position_speed_rpm = 500U,
    .acceleration = 20U,
    .feedback_period_ms = 20U,
    .feedback_timeout_ms = 100U,
    .auto_enable = true,
};

if (!gimbal.init(&gimbal, &gimbal_config))
{
    /* 初始化失败处理 */
}
```

`feedback_period_ms` 非0时：

- `update()` 按该周期主动查询两个电机的位置；
- 两个电机按该周期自动返回转速；
- 电机的位置自动回传会被关闭，避免主动查询与自动回传重复占用CAN总线。

若将其设置为0，组件不会主动查询位置，也不会启用转速自动回传。此时
`yaw_deg`、`pitch_deg`、`yaw_speed_dps` 和 `pitch_speed_dps` 只会使用电机对象中
已有的反馈缓存，通常不适合需要实时反馈和限位保护的运行场景。

`feedback_timeout_ms` 用于判断双轴反馈是否超时。建议该值大于
`feedback_period_ms`，并至少允许数个反馈周期，例如反馈周期为20 ms时可先使用100 ms。

## 5. 角度控制

同时设置水平和俯仰角度：

```c
gimbal.set_angle(&gimbal, 30.0f, -10.0f);
```

只修改一个轴：

```c
gimbal.set_yaw_angle(&gimbal, 45.0f);
gimbal.set_pitch_angle(&gimbal, 20.0f);
```

角度命令超过机械范围时会自动限制到 `min_angle_deg` 或 `max_angle_deg`，并令：

```c
gimbal_data.target_limited == true
```

两个轴的位置命令先进入电机同步缓存，随后通过广播同步触发，因此启动时间接近一致。

## 6. 角速度控制

```c
/* 水平轴30 deg/s，俯仰轴-10 deg/s */
gimbal.set_angular_velocity(&gimbal, 30.0f, -10.0f);
```

换算关系：

```text
motor_rpm = axis_speed_deg_per_s * motor_to_axis_ratio / 6
```

如果某轴已位于机械限位，继续向限位外运动的速度会被限制为0。角速度模式下必须周期调用 `update()`；检测到轴越过限位或反馈通信超时时，组件会同步停止两个轴。启用 `feedback_timeout_ms` 后，在收到双轴有效反馈前不会接受角速度命令。

## 7. 停止、使能和回零

```c
/* 依次立即停止两个轴，但保持电机使能 */
gimbal.stop(&gimbal);

/* 失能和重新使能 */
gimbal.enable(&gimbal, false);
gimbal.enable(&gimbal, true);

/* 根据两个轴各自的home_mode同步回零 */
gimbal.home(&gimbal);
```

将两个轴当前位置清零：

```c
gimbal.zero(&gimbal);
```

`zero()` 会向两个电机发送当前位置清零命令，并把组件内部的 `position_offset_deg` 重置为0。该操作不会写入电机的单圈回零参数。

## 8. 周期更新任务

建议以5~10 ms周期调用：

```c
void gimbalTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        gimbal.update(&gimbal);
        osDelay(10U);
    }
}
```

`update()` 负责：

- 按 `feedback_period_ms` 主动查询两个电机的位置
- 将电机位置换算为云台角度
- 将电机RPM换算为云台角速度
- 更新通信超时状态
- 汇总双轴动作完成状态
- 在角速度模式下执行机械限位保护

## 9. 获取运行状态

```c
GimbalData_t gimbal_data;

gimbal.get_data(&gimbal, &gimbal_data);
```

建议业务代码通过 `get_data()` 获取一致快照，再读取：

```c
float yaw_deg = gimbal_data.yaw_deg;
float pitch_deg = gimbal_data.pitch_deg;
```

`Gimbal_t` 中只有一层 `data` 成员。调试器直接观察对象时，应使用：

```text
gimbal.data.yaw_deg
gimbal.data.pitch_deg
```

不存在 `gimbal.data.data.yaw_deg` 或 `gimbal.data.data.pitch_deg`。

常用字段：

```text
yaw_deg / pitch_deg                 当前双轴角度
yaw_speed_dps / pitch_speed_dps     当前双轴角速度
target_yaw_deg / target_pitch_deg   限位后的目标角度
enabled                             双轴使能状态
communication_ok                    双轴反馈均未超时
motion_complete                     双轴动作均已完成
target_limited                      最近命令触发了机械限位
control_mode                        当前控制模式
```

其中 `target_yaw_deg` 和 `target_pitch_deg` 是成功下发并经过软件限位后的目标值；
`yaw_deg` 和 `pitch_deg` 是根据电机位置反馈换算得到的实际值。电机能够运动只表示
控制命令已经下发成功，并不能证明位置反馈已经正常接收。

## 10. 反馈异常排查

如果电机能够运动，但 `gimbal.data.yaw_deg` 或 `gimbal.data.pitch_deg` 一直为0，依次观察：

```text
gimbal.data.update_count
zdt42_motor_yaw.data.rx_frame_count
zdt42_motor_pitch.data.rx_frame_count
zdt42_motor_yaw.data.position_deg
zdt42_motor_pitch.data.position_deg
gimbal.data.communication_ok
```

- `update_count` 不增加：周期任务没有调用 `gimbal.update()`。
- 对应电机的 `rx_frame_count` 不增加：位置查询响应没有进入CAN接收回调，应检查电机地址、CAN过滤器、接收FIFO中断和接线。
- `rx_frame_count` 增加但电机 `position_deg` 始终为0：检查位置反馈功能码和数据解析。
- 电机 `position_deg` 正常但云台角度不正确：检查 `motor_to_axis_ratio`、`position_offset_deg` 和 `reversed`。
- `communication_ok` 为 `false`：至少一个轴在 `feedback_timeout_ms` 内没有收到有效反馈。

电机对象名称由应用层实例定义，上述 `zdt42_motor_yaw` 和 `zdt42_motor_pitch` 仅为示例。

## 11. 使用约束

- 云台初始化前，两个电机对象必须已经初始化。
- 同一个电机对象不能同时绑定为水平轴和俯仰轴。
- 两个电机必须使用不同CAN地址。
- 机械限位是软件保护，首次运行仍应使用低速并准备断电。
- 主动位置查询、转速回传和控制命令共用CAN总线，反馈周期不宜设置过短，建议从20 ms开始。
- `Gimbal_t` 不负责电机供电、限位开关硬件保护或急停回路。
