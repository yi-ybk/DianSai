# Motor 电机模块使用说明

## 1. 模块功能

`motor_driver` 统一封装以下设备：

- 单 PWM 单方向减速电机；
- 双 PWM 正反转减速电机；
- TB6612 的 PWM + IN1/IN2/STBY 减速电机；
- PWM 舵机；
- 可选绑定独立 `Encoder_t` 获取转速反馈。

Motor 只负责执行电机输出和同步编码器转速，不包含速度 PID。需要闭环线速度时由 `Wheel_t` 组合 Motor、Encoder 和 Pid。

## 2. 初始化顺序

```text
MX_TIMx_Init / MX_GPIO_Init
    -> EncoderInit（需要反馈时）
        -> MotorInit
```

编码器对象必须在 `MotorInit()` 前完成初始化。PWM 的 `period` 单位是秒，必须与定时器实际 PWM 周期一致。

## 3. TB6612 减速电机

先在 CubeMX 中配置：

- PWMA/PWMB 对应定时器 PWM 通道；
- IN1、IN2 和可选 STBY 为 GPIO Output；
- STBY 默认建议拉低，方向引脚默认低电平。

```c
#include "motor_driver.h"
#include "tim.h"

static Motor_t left_motor = { MOTOR_OBJECT_DEFAULT };

static const MotorInitConfig_t left_motor_config = {
    .type = MOTOR_TYPE_REDUCTION,
    .pwm = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_1,
        .period = 0.001f,
        .init_duty = 0.0f,
    },
    .use_reverse_pwm = false,
    .use_tb6612 = true,
    .tb6612 = {
        .in1 = {GPIOB, GPIO_PIN_0},
        .in2 = {GPIOB, GPIO_PIN_1},
        .standby = {GPIOB, GPIO_PIN_10},
        .use_standby = true,
        .brake_on_stop = false,
        .reversed = false,
    },
    .use_encoder = true,
    .encoder = &left_encoder,
    .min_duty = 0.10f,
    .max_duty = 1.00f,
    .init_output = 0.0f,
};

bool LeftMotorInit(void)
{
    return left_motor.init(&left_motor, &left_motor_config);
}
```

控制：

```c
left_motor.set_speed(&left_motor, 0.5f);   /* 正转 */
left_motor.set_speed(&left_motor, -0.5f);  /* 反转 */
left_motor.set_speed(&left_motor, 0.0f);   /* 停止输出 */
left_motor.stop(&left_motor);              /* 停止PWM和电机 */
left_motor.start(&left_motor);             /* 重新使能 */
```

TB6612 模式的输入范围为 `-1.0f` 到 `1.0f`。`reversed` 交换逻辑正反转；`brake_on_stop=true` 时停止方向为 IN1=IN2=高电平，否则滑行并可拉低 STBY。

## 4. 双 PWM 减速电机

```c
static const MotorInitConfig_t motor_config = {
    .type = MOTOR_TYPE_REDUCTION,
    .pwm = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_1,
        .period = 0.001f,
    },
    .reverse_pwm = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_2,
        .period = 0.001f,
    },
    .use_reverse_pwm = true,
    .use_tb6612 = false,
    .min_duty = 0.0f,
    .max_duty = 1.0f,
    .init_output = 0.0f,
};
```

正转时主 PWM 输出、反向 PWM 为 0；反转时相反。不要把同一个定时器通道注册给多个 Motor/PWM 对象。

单 PWM 且不使用 TB6612 时，负输出会被限制为 0，只支持单方向。

## 5. 舵机

```c
static Motor_t steering_servo = { MOTOR_OBJECT_DEFAULT };

static const MotorInitConfig_t servo_config = {
    .type = MOTOR_TYPE_SERVO,
    .pwm = {
        .htim = &htim3,
        .channel = TIM_CHANNEL_1,
        .period = 0.020f,
    },
    .servo_min_angle = 0.0f,
    .servo_max_angle = 180.0f,
    .servo_min_pulse_us = 500.0f,
    .servo_max_pulse_us = 2500.0f,
    .init_output = 90.0f,
};

void ServoSetAngle(float angle_deg)
{
    steering_servo.set_angle(&steering_servo, angle_deg);
}
```

角度会被限制到配置范围，再线性映射到脉宽。具体舵机的安全脉宽范围应以实物规格为准，首次调试建议缩小范围。

## 6. 编码器更新与数据

```c
MotorData_t data;

left_motor.update(&left_motor, 0.01f);
left_motor.get_data(&left_motor, &data);
```

绑定编码器后，`MotorUpdate()` 会调用 `EncoderUpdate()`；不要再从另一个任务重复更新同一个 Encoder。Motor 只对外同步 `speed_rps`，需要累计计数和原始方向时通过 Encoder 获取。

`MotorData_t.output` 对减速电机是归一化输出，对舵机是目标角度；两种类型不能按同一物理单位解释。

## 7. 初始化和停止行为

`MotorInit()` 当前会注册 PWM、调用 `MotorStart()` 并下发 `init_output`。即使上层 Wheel 的 `auto_start=false`，Motor 自身也可能已经处于使能状态，因此安全初始化应把 `init_output` 设为 0，并在系统未准备好时主动调用 `MotorStop()`。

## 8. 常见问题

- 初始化失败：检查 PWM 句柄、通道、周期、编码器初始化状态和 TB6612 GPIO。
- 电机方向相反：优先只修改 TB6612 的 `reversed` 或 Wheel 的 `reversed` 其中一层。
- 小输出不转：根据静摩擦调整 `min_duty`，但过大可能导致启动突跳。
- 编码器速度为零：确认 Encoder 已初始化并启动，且周期调用 `MotorUpdate()`。
- 舵机抖动：检查 PWM 周期、时钟、脉宽范围、供电和共地。
- `MotorStop()` 后无输出：重新调用 `MotorStart()`，再设置目标输出。
