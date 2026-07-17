# IMU 模块使用说明

## 1. 模块结构

IMU 目录分为两层：

- `imu_driver.c/.h`：通用 UART DMA 接收、软件环形队列、定长帧同步、数据快照接口；
- `mg354pdh0_driver.c/.h`：M-G354PDH0 的设备初始化命令和 22 字节 Burst 解析器。

通用层通过 `ImuFrameParser_t` 和 `ImuDeviceInit_t` 函数指针适配不同型号。接收队列满时会丢弃最旧字节，解析器会重新搜索帧头恢复同步。

## 2. 依赖与串口配置

需要包含：

```text
module/imu/imu_driver.c
module/imu/mg354pdh0_driver.c
bsp/usart/bsp_usart.c
```

M-G354PDH0 当前配置要求：

```text
UART: 460800, 8N1
DMA RX: Circular
UART global interrupt: Enabled
输出模式: 125 SPS, 16 bit, Auto Mode
帧长度: 22 bytes
```

调用 IMU 初始化前必须完成 `MX_DMA_Init()` 和对应的 `MX_USARTx_UART_Init()`。M-G354PDH0 使用 3.3 V TTL，MCU TX 接 IMU RX，MCU RX 接 IMU TX，并确保共地。

## 3. M-G354PDH0 对象定义

协议帧头、帧尾数组的生命周期必须覆盖 IMU 对象，推荐定义为静态数组或文件作用域复合字面量：

```c
#include "imu_driver.h"
#include "mg354pdh0_driver.h"
#include "usart.h"

static const uint8_t imu0_header[] = {MG354PDH0_FRAME_HEADER};
static const uint8_t imu0_tail[] = {MG354PDH0_FRAME_TAIL};
static ImuQuaternionEkfConfig_t imu0_ekf_config =
    IMU_QUATERNION_EKF_CONFIG_DEFAULT(0.008f);

static Imu_t imu0 = {
    IMU_OBJECT_DEFAULT,
    .attitude_solver = {
        .init = ImuQuaternionEkfSolverInit,
        .update = ImuQuaternionEkfSolverUpdate,
        .context = &imu0_ekf_config,
    },
    .protocol_frame_len = MG354PDH0_FRAME_LEN,
    .protocol_header_bytes = imu0_header,
    .protocol_header_len = sizeof(imu0_header),
    .protocol_tail_bytes = imu0_tail,
    .protocol_tail_len = sizeof(imu0_tail),
    .init_config = {
        .recv_buff_size = IMU_UART_DMA_RX_BUFFER_LEN,
        .usart_handle = &huart1,
        .parser = Mg354pdh0FrameParse,
        .parser_context = NULL,
        .device_init = Mg354pdh0DeviceInit,
        .device_context = NULL,
    },
};
```

## 4. FreeRTOS 解析任务

`Mg354pdh0DeviceInit()` 内部使用 `vTaskDelay()` 等待传感器上电并发送配置命令，因此应在调度器启动后的任务中初始化，不要在 `main()` 启动调度器前调用。

```c
static void ImuTask(void *argument)
{
    Imu_t *imu = (Imu_t *)argument;

    if ((imu == NULL) ||
        (imu->init == NULL) ||
        !imu->init(imu, &imu->init_config))
    {
        osThreadExit();
    }

    for (;;)
    {
        imu->process(imu);
        osDelay(1U);
    }
}

void CreateImuTask(void)
{
    static const osThreadAttr_t attributes = {
        .name = "imu0Task",
        .stack_size = 512U,
        .priority = osPriorityAboveNormal,
    };

    (void)osThreadNew(ImuTask, &imu0, &attributes);
}
```

每个 UART 只能注册给一个 BSP USART 实例。不要让多个模块同时注册同一个 UART 句柄。

## 5. 获取数据

```c
ImuData_t snapshot;

imu0.get_data(&imu0, &snapshot);

float ax_mps2 = snapshot.accel.x;
float gz_radps = snapshot.gyro.z;
uint32_t valid_frames = snapshot.frame_count;
```

也可只读取某类物理量：

```c
ImuVector3f_t accel;
ImuVector3f_t gyro;

imu0.get_accel(&imu0, &accel);
imu0.get_gyro(&imu0, &gyro);
```

`get_data()` 在临界区中复制 `accel/gyro/angle/quaternion/frame_count`，用于获得同一时刻的一致快照。`frame_count` 只在型号解析器返回成功时递增，可用于判断数据是否更新。

当前统一单位：

| 数据 | 单位 |
| --- | --- |
| `accel` | `m/s^2` |
| `gyro` | `rad/s` |
| `angle.x`、`angle.y` | 姿态解算得到的横滚、俯仰角，单位为度 |
| `angle.z` | 姿态解算得到的相对偏航角，单位为度，范围为 `[-180, 180]` |
| `quaternion` | 无量纲 |

M-G354PDH0 当前 Burst 帧只直接提供加速度和角速度。需要姿态解算时，可在 `Imu_t.attitude_solver` 中绑定 Mahony 或 QuaternionEKF 解算器。不配置 `attitude_solver` 时只解析原始数据。当前 `imu0` 使用 `ImuMahonySolverInit()` 和 `ImuMahonySolverUpdate()`。

姿态解算器在每个成功帧上更新 `angle/quaternion`，并在 `frame_count` 递增前完成，因此 `get_data()` 返回的原始数据、姿态数据和帧计数属于同一次完整更新。不同型号的解析器需要先将角速度统一为 `rad/s`、加速度统一为 `m/s^2`。Mahony 状态存放在各自配置实例中，可以绑定多个 IMU；QuaternionEKF 底层仍为全局单实例。

`angle.x/angle.y/angle.z` 均由 Mahony 输出四元数转换得到，因此欧拉角与 `quaternion` 表示同一个姿态。在俯仰角接近 `+90°/-90°` 时仍存在欧拉角奇异性，内部会将反三角函数输入限制到合法范围以避免浮点误差产生 `NaN`。

`ImuMahonyConfig_t.accel_gravity_sign` 仅用于姿态解算器内部的重力方向校正，取值为 `1.0f` 或 `-1.0f`；保留零初始化兼容性，值为 `0.0f` 时按 `1.0f` 处理。它不会改变 `get_accel()` 返回的原始加速度。Mahony 默认以静止时 `+Z` 重力方向为观测；M-G354PDH0 当前安装方向静止时约为 `-Z`，因此 `imu0` 配置为 `-1.0f`。

Mahony 的 `proportional_gain` 控制重力方向修正速度，当前使用 `0.5f`；值过大时动态加速度更容易扰动姿态，值过小时横滚和俯仰收敛较慢。启动标定已校正三轴陀螺仪零偏，因此当前 `integral_gain` 设为 `0.0f`，避免持续线性加速度造成积分累积。

M-G354PDH0 可通过 `Mg354pdh0GyroCalibration_t` 传入 `parser_context` 启用启动静止标定。标定帧只累积陀螺仪零偏，不进入姿态解算；完成后第一帧才开始更新 `frame_count`、姿态和四元数。标定期间必须保持 IMU 静止。仅使用陀螺仪和加速度计无法长期约束偏航角，启动零偏标定只能降低而不能消除 `angle.z` 的长期漂移；需要绝对航向时应增加磁力计或其他航向观测。

偏航零偏可选自动或固定参数模式。当前 `imu0` 使用自动模式：

```c
static Mg354pdh0GyroCalibration_t imu0_gyro_calibration =
    MG354PDH0_GYRO_CALIBRATION_DEFAULT(250U);
```

若已在静止条件下测得 `gyro.z` 的零偏，可替换为固定模式：

```c
static Mg354pdh0GyroCalibration_t imu0_gyro_calibration =
    MG354PDH0_GYRO_CALIBRATION_FIXED_YAW_DEFAULT(0.0040f);
```

固定参数单位为 `rad/s`，驱动会执行 `gyro.z -= gyro_bias.z`；因此静止时原始 `gyro.z` 为 `+0.0040 rad/s` 时应传入 `+0.0040f`。固定模式不会等待启动标定，也不会校正 X/Y 轴零偏。

## 6. 适配其他 IMU

新增型号时需要实现：

```c
bool DeviceInit(Imu_t *imu, void *context);

uint8_t DeviceFrameParse(Imu_t *imu,
                         const uint8_t *frame,
                         uint16_t frame_len,
                         void *context);
```

并为对象配置固定帧长、帧头和帧尾。解析器成功返回 `1U`，校验或格式错误返回 `0U`。不要在解析失败时更新 `imu->data`。

当前通用层的 `ImuVerifyFrame()` 没有实现校验和算法。如果目标协议启用 CRC/Checksum，应在型号解析器中校验后再写入数据。

## 7. M-G354PDH0 注意事项

- 初始化固定等待 1 秒，然后发送文档中的最小 Auto Mode 配置序列；
- 当前不读取 `NOT_READY` 或诊断寄存器；
- 当前配置关闭 Checksum；
- 数据按大端有符号 16 位解析；
- 加速度比例为 `0.0002 g/LSB`，转换为 `m/s^2`；
- 角速度比例为 `0.016 deg/s/LSB`，转换为 `rad/s`；
- 更完整的寄存器和帧定义参见 `M-G354PDH0_UART_使用指南.md`。

## 8. 常见问题

- `ImuInit()` 返回 `false`：检查协议长度、帧头/尾、UART 句柄、DMA 缓冲长度和设备初始化发送结果。
- `frame_count` 不增长：检查波特率、TX/RX 交叉、DMA/IRQ、Auto Mode 配置和 22 字节帧格式。
- 持续丢帧或队列溢出：提高 `process()` 调用频率，避免更高优先级任务长期阻塞 IMU 任务。
- 数据方向相反：确认传感器安装坐标系，在应用层统一坐标变换，不要直接修改原始比例系数。
- 数据读取偶发不一致：应使用 `get_data()`，不要跨任务直接逐字段读取 `imu->data`。
