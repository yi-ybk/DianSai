# 张大头 ZDT_X42S CAN 驱动使用说明

## 1. 模块功能

本模块用于通过 Classic CAN 控制张大头 ZDT_X42S 第二代闭环步进电机，提供：

- 电机使能、失能和立即停止
- 有符号速度模式控制
- 梯形位置模式和快速位置模式
- 回零、设置零点和当前位置清零
- 多机同步触发
- 实时转速、位置及其他系统参数查询
- 定时反馈配置
- 厂家长命令自动 CAN 分包
- 反馈校验、解析和一致快照读取

驱动使用 CAN 扩展数据帧：

```text
ExtId = (motor_id << 8) | packet_index
```

数据区不包含电机地址。长命令每个分包的首字节都会重复功能码。

## 2. 硬件与电机配置

电机建议配置为：

```text
P_Serial = CAN1_MAP
CAN_Baud = 500000
ID_Addr  = 1
Checksum = 0x6B
Response = Receive 或 Both
```

MSPM0G3507 与电机之间必须使用 3.3V 逻辑电平的 CAN 收发器，不能将 MCU 引脚直接连接 CANH/CANL。CANH/CANL 总线两端各接 `120Ω` 终端电阻并共地。

当前工程通过 `DianSai_MSPM0G3507.syscfg` 配置 `CANFD0`：

```text
CANTX     = PA26
CANRX     = PA27
CANCLK    = 80 MHz (SYSPLLCLK1)
Baud rate = 500 kbit/s Classic CAN
RX FIFO   = FIFO0，8个8字节元素
```

## 3. 初始化

`SYSCFG_DL_init()` 应在创建电机对象前完成。对象可定义在 `application/robot.c`：

```c
#include "ti_msp_dl_config.h"
#include "zhangdatou_42.h"

Zdt42_t stepper = {ZDT42_OBJECT_DEFAULT};

static const Zdt42InitConfig_t stepper_config =
    ZDT42_INIT_CONFIG_DEFAULT(MCAN_GIMBAL_INST, 1U);

void UserRobotInit(void)
{
    if (!stepper.init(&stepper, &stepper_config))
    {
        /* 初始化失败处理 */
    }
}
```

每个电机地址对应一个独立 `Zdt42_t` 对象。同一 CAN 总线上的电机地址不能重复。
当前工程的 `MCAN_GIMBAL_INST_IRQHandler()` 已把 CANFD0 中断转发给 BSP；首个电机注册时会自动使能对应 NVIC 中断。多个电机对象可共享同一 CANFD0 和 FIFO0。

## 4. 基本控制

```c
/* 立即使能 */
stepper.enable(&stepper, true, false);

/* CW方向1000 RPM，加速度20，立即执行 */
stepper.set_speed(&stepper, 1000.0f, 20U, false);

/* CCW方向500 RPM */
stepper.set_speed(&stepper, -500.0f, 20U, false);

/* 立即停止 */
stepper.stop(&stepper, false);

/* 失能 */
stepper.enable(&stepper, false, false);
```

`set_speed()` 的正值表示 CW，负值表示 CCW，范围会限制到 `-5000~5000 RPM`。厂家协议中加速度为 `0~255`，其中 `0` 表示直接启动。

## 5. 位置控制

### 5.1 梯形位置模式

```c
/* 以600 RPM、加速度20，相对当前位置正向运动3200脉冲 */
stepper.move_position(&stepper,
                      3200,
                      600U,
                      20U,
                      ZDT42_POSITION_RELATIVE_CURRENT,
                      false);

/* 反向运动3200脉冲 */
stepper.move_position(&stepper,
                      -3200,
                      600U,
                      20U,
                      ZDT42_POSITION_RELATIVE_CURRENT,
                      false);
```

位置参考类型：

| 枚举 | 含义 |
|---|---|
| `ZDT42_POSITION_RELATIVE_TARGET` | 相对上一输入目标位置 |
| `ZDT42_POSITION_ABSOLUTE` | 绝对位置 |
| `ZDT42_POSITION_RELATIVE_CURRENT` | 相对当前电机实时位置 |

### 5.2 快速位置模式

快速位置模式应先设置运动参数，再下发带符号脉冲数：

```c
stepper.set_quick_position(&stepper,
                           600U,
                           20U,
                           ZDT42_POSITION_RELATIVE_CURRENT,
                           false);
stepper.move_quick(&stepper, 3200);
```

## 6. 读取反馈

请求一次实时速度和位置：

```c
stepper.read_parameter(&stepper, ZDT42_PARAM_SPEED);
stepper.read_parameter(&stepper, ZDT42_PARAM_POSITION);
```

获取解析后的数据快照：

```c
Zdt42Data_t stepper_data;

stepper.get_data(&stepper, &stepper_data);

/* stepper_data.speed_rpm   ：有符号转速，单位RPM */
/* stepper_data.position_deg：有符号多圈位置，单位deg */
```

反馈格式来自厂家例程：

```text
速度：35 + 方向 + uint16 RPM + 6B
位置：36 + 方向 + uint32位置 + 6B
位置角度 = uint32位置 * 360 / 65536
```

设置每 20 ms 自动返回实时速度：

```c
stepper.set_auto_return(&stepper, ZDT42_PARAM_SPEED, 20U);
```

停止自动返回：

```c
stepper.set_auto_return(&stepper, ZDT42_PARAM_SPEED, 0U);
```

## 7. 回零与清零

```c
/* 触发单圈就近回零 */
stepper.home(&stepper, ZDT42_HOME_NEAREST, false);

/* 强制中断回零 */
stepper.interrupt_home(&stepper);

/* 将当前位置设置为单圈零点并保存 */
stepper.set_origin(&stepper, true);

/* 将当前位置直接清零 */
stepper.reset_position(&stepper);

/* 解除堵转、过热和过流保护 */
stepper.clear_fault(&stepper);
```

## 8. 多机同步

先分别向电机发送 `sync=true` 的运动命令，最后由任意已初始化对象发送广播触发：

```c
motor1.set_speed(&motor1, 500.0f, 20U, true);
motor2.set_speed(&motor2, -500.0f, 20U, true);
motor1.trigger_sync(&motor1);
```

广播扩展 ID 为 `0x0000`，数据为 `FF 66 6B`。

## 9. 原始命令扩展

当公开接口未覆盖厂家命令时，可发送不含地址的原始命令：

```c
const uint8_t command[] = {0x35U, 0x6BU};

stepper.send_raw(&stepper, 1U, command, sizeof(command));
```

要求：

- `command[0]` 是功能码。
- 最后一个字节是电机当前配置的校验字节。
- 不要在数据中放入电机地址。
- 超过 8 字节时驱动自动分包并在每包重复功能码。

## 10. 回调和并发约束

反馈回调运行于 CAN 接收中断上下文，只适合设置标志、复制少量数据或发送任务通知：

```c
static void StepperFeedbackCallback(Zdt42_t *motor,
                                    const Zdt42Data_t *data,
                                    void *context)
{
    (void)motor;
    (void)data;
    (void)context;
}

stepper.set_callback(&stepper, StepperFeedbackCallback, NULL);
```

不要在回调中执行阻塞等待、OLED刷新或日志格式化。`get_data()` 使用临界区复制一致快照。

单个对象内部复用一个 CAN 发送缓冲区，因此同一对象的发送接口应由一个任务调用；多任务访问时应在应用层增加互斥锁。

## 11. 反馈状态说明

`Zdt42Data_t` 中：

- `command_received=true`：收到 `0x02`，仅表示命令接收成功。
- `motion_complete=true`：收到 `0x9F`，表示动作执行完成。
- `raw_data/raw_length`：保留最近一帧原始反馈，供未内置解析的功能使用。
- `checksum_error_count`：收到的末尾校验字节与配置不一致。
- `tx_error_count`：CAN 邮箱等待或发送失败。

需要判断位置运动或回零完成时，应将电机 `Response` 配置为 `Both` 或 `Reached`，不能将 `0x02` 当作运动完成。
