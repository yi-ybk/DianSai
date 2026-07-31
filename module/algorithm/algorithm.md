# Algorithm 算法模块使用说明

## 1. 模块内容

`module/algorithm` 提供以下通用算法：

| 文件 | 功能 |
| --- | --- |
| `controller.c/.h` | 带多种优化环节、自动测量周期的高级位置式 PID |
| `crc8.c/.h` | CRC8 计算和逐字节更新 |
| `crc16.c/.h` | CRC16、Modbus CRC16 和逐字节更新 |
| `kalman_filter.c/.h` | 基于 CMSIS-DSP 矩阵运算的通用卡尔曼滤波器 |
| `user_lib.c/.h`、`user_lib_math.h` | 限幅、角度处理、向量和矩阵辅助函数；轻量数值接口单独由 `user_lib_math.h` 提供 |

本目录的 `controller` 与 `module/pid` 是两套独立 PID 实现。新模块需要显式传入 `dt_s`、对象式接口和运行快照时，优先使用 `module/pid`；需要梯形积分、变速积分、输出/微分滤波或 DWT 自动计时时使用 `controller`。

## 2. 工程依赖

算法模块依赖 CMSIS-DSP；`controller` 还依赖 `bsp_dwt`，卡尔曼滤波器会动态申请内存。工程至少需要相应头文件路径和 CMSIS-DSP 库配置。

不要在中断中首次初始化卡尔曼滤波器。当前实现没有释放接口，初始化函数应只调用一次。

## 3. 高级 PID controller

```c
#include "controller.h"

static PIDInstance speed_pid;

void SpeedPidInit(void)
{
    PID_Init_Config_s config = {
        .Kp = 1.0f,
        .Ki = 0.2f,
        .Kd = 0.01f,
        .MaxOut = 1.0f,
        .DeadBand = 0.01f,
        .Improve = PID_Integral_Limit |
                   PID_Derivative_On_Measurement,
        .IntegralLimit = 0.5f,
    };

    PIDInit(&speed_pid, &config);
}

float SpeedPidUpdate(float measured_speed, float target_speed)
{
    return PIDCalculate(&speed_pid, measured_speed, target_speed);
}
```

`PIDCalculate()` 使用 `DWT_GetDeltaT()` 自动取得两次调用之间的周期，因此必须先初始化 DWT，并按稳定周期调用。误差进入 `DeadBand` 后，当前输出和本次积分项会清零。

可组合的 `Improve` 标志包括积分限幅、微分先行、梯形积分、比例先行、输出滤波、变速积分、微分滤波和错误检测。启用滤波或变速积分时，还要配置对应的 `CoefA`、`CoefB`、`Output_LPF_RC`、`Derivative_LPF_RC`。

## 4. CRC

一次性计算：

```c
#include "crc8.h"
#include "crc16.h"

uint8_t payload[] = {0x01U, 0x02U, 0x03U};
uint8_t crc8_value = crc_8(payload, sizeof(payload));
uint16_t crc16_value = crc_16(payload, sizeof(payload));
uint16_t modbus_crc = crc_modbus(payload, sizeof(payload));
```

流式计算：

```c
uint8_t crc = CRC_START_8;

for (uint16_t i = 0U; i < data_len; ++i)
    crc = update_crc_8(crc, data[i]);
```

CRC 结果是否需要高低字节交换由具体通信协议决定。

## 5. 通用卡尔曼滤波器

以下示例建立一个无控制量的一维滤波器：

```c
#include "kalman_filter.h"

static KalmanFilter_t value_kf = {0};

void ValueFilterInit(void)
{
    Kalman_Filter_Init(&value_kf, 1U, 0U, 1U);

    value_kf.F_data[0] = 1.0f;
    value_kf.H_data[0] = 1.0f;
    value_kf.P_data[0] = 1.0f;
    value_kf.Q_data[0] = 0.01f;
    value_kf.R_data[0] = 0.10f;
    value_kf.StateMinVariance[0] = 0.0001f;
}

float ValueFilterUpdate(float measurement)
{
    value_kf.MeasuredVector[0] = measurement;
    return Kalman_Filter_Update(&value_kf)[0];
}
```

`xhatSize`、`uSize`、`zSize` 分别是状态、控制和观测维度。初始化后必须填写与模型对应的 `F`、`B`、`H`、`Q`、`R`、`P` 数据；矩阵按一维行主序数组保存。

注意事项：

- `KalmanFilter_t` 应先清零，建议定义为静态对象；
- 初始化会进行多次堆内存分配，当前没有内存不足检查和释放接口；
- `Kalman_Filter_Update()` 返回内部 `FilteredValue` 指针，不要释放；
- `MatStatus` 保存最近一次 CMSIS-DSP 矩阵运算状态，滤波异常时应检查它；
- 使用 `User_Func0_f` 到 `User_Func6_f` 和 `SkipEq1` 到 `SkipEq5` 可以替换标准步骤，但需理解滤波方程后再使用。

## 6. user_lib

常用接口示例：

```c
#include "user_lib_math.h"

float limited = float_constrain(command, -1.0f, 1.0f);
```

简单浮点绝对值统一使用标准库 `fabsf()`。需要角度、向量或矩阵辅助接口时包含完整头文件：

```c
#include "user_lib.h"

float angle_deg = theta_format(raw_angle_deg);
float angle_rad = rad_format(raw_angle_rad);
float length = NormOf3d(vector);
float average = AverageFilter(sample, sample_buffer, BUFFER_LEN);
```

PID、Motor 和 Wheel 的限幅统一复用 `float_constrain()`。`user_lib_math.h` 不引入 RTOS、STM32 或 CMSIS-DSP 头文件，适合底层模块使用。

使用限制：

- `Norm3d()` 会原地修改向量，零向量会发生除零；
- `AverageFilter()` 要求缓冲区有效且 `len > 0`；
- `MatInit()` 和 `zmalloc()` 使用堆内存，当前没有释放接口和空指针检查；
- `loop_float_constrain()` 通过循环归一化，输入值跨度很大时耗时会增加；
- 这些函数没有内部锁，多任务共享可变缓冲区时由调用方同步。

## 7. 选择建议

| 需求 | 推荐接口 |
| --- | --- |
| 普通速度或位置闭环、显式周期 | `module/pid/pid.h` |
| 需要高级 PID 优化环节、DWT 自动周期 | `controller.h` |
| 通信数据校验 | `crc8.h`、`crc16.h` |
| 自定义状态空间滤波 | `kalman_filter.h` |
| 简单限幅、向量和角度工具 | `user_lib.h` |
