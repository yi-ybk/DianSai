# M-G354PDH0 UART 使用指南

> 适用器件：Epson M-G354PDH0  
> 接口：3.3 V TTL UART  
> 文档依据：M-G354PDH0 Data Sheet Rev.20160212

## 1. 使用前结论

- UART 为 **3.3 V TTL/LVCMOS 电平**，不是 RS-232。
- UART 与 SPI 只能选择一种连接，不能同时接入。
- 上电或复位后，最长需要约 `800 ms` 完成内部初始化。
- 推荐优先使用：
  - 调试：UART Manual Burst
  - 连续采集：UART Auto Mode
- 数据均为二进制格式，不是 ASCII 文本。
- 传感器数据采用大端序（Big Endian）和有符号二进制补码。

## 2. 硬件连接

### 2.1 引脚定义

| IMU引脚 | 名称 | 方向 | 连接 |
|---:|---|---|---|
| 7 | SOUT | 输出 | 接主控/USB-UART RX |
| 9 | SIN | 输入 | 接主控/USB-UART TX |
| 13 | DRDY/GPIO1 | I/O | 可选，数据就绪信号 |
| 16 | `/RST` | 输入 | 不使用时保持高电平 |
| 10、11、12 | VCC | 电源 | 全部接3.3 V |
| 3、4、8、15 | GND | 电源 | 全部接地 |
| 17～20 | NC | — | 禁止连接 |

### 2.2 典型连接

```text
USB-UART TX  ─────> IMU SIN   Pin 9
USB-UART RX  <───── IMU SOUT  Pin 7
USB-UART GND ─────  IMU GND
3.3 V电源     ─────  IMU VCC
```

注意：

- USB-UART 模块必须支持 `3.3 V TTL`。
- 不允许直接连接传统 ±12 V RS-232 接口。
- VCC 推荐范围为 `3.15～3.45 V`。
- `/RST` 不使用时保持高电平。
- 未使用输入引脚建议通过电阻上拉至 VCC。

## 3. 串口参数

```text
波特率：460800 或 230400 baud
数据位：8
停止位：1
校验位：None
流控：None
格式：8N1
命令结束符：0x0D
```

推荐默认尝试：

```text
460800 baud
```

如果通信失败，再尝试：

```text
230400 baud
```

原因：UART 配置可以保存到非易失存储器，器件可能已经被修改过波特率。

## 4. 寄存器窗口

器件寄存器分为两个 Window。

```text
Window 0：状态、模式、传感器数据
Window 1：采样率、滤波器、UART、Burst配置
```

切换窗口：

```text
FE 00 0D    # 切换到Window 0
FE 01 0D    # 切换到Window 1
```

其中：

```text
0xFE = 0x80 | 0x7E
```

即向 `WIN_CTRL(0x7E)` 写入窗口号。

## 5. UART命令格式

### 5.1 读取寄存器

发送格式：

```text
[偶地址] [任意值] [0x0D]
```

响应格式：

```text
[地址] [高字节] [低字节] [0x0D]
```

示例：读取 Window 0 的 `MODE_CTRL(0x02)`：

```text
TX: 02 00 0D
RX: 02 04 00 0D
```

解析：

```text
MODE_CTRL = 0x0400
```

要求：

- 读取地址必须是偶数。
- 返回数据固定为16 bit。
- 高字节在前。

### 5.2 写入寄存器

发送格式：

```text
[0x80 | 字节地址] [数据] [0x0D]
```

写命令通常无响应。

示例：向地址 `0x03` 写入 `0x01`：

```text
TX: 83 01 0D
```

含义：

```text
MODE_CTRL高字节 = 0x01
进入Sampling Mode
```

返回 Configuration Mode：

```text
TX: 83 02 0D
```

## 6. 上电初始化流程

推荐流程：

```text
1. 3.3 V上电
2. 等待至少800 ms
3. 切换到Window 1
4. 轮询GLOB_CMD.NOT_READY
5. 确认NOT_READY == 0
6. 切换到Window 0
7. 读取DIAG_STAT
8. 确认HARD_ERR == 0
9. 保持Configuration Mode
10. 配置采样率、滤波器、UART和Burst
11. 进入Sampling Mode
```

### 6.1 轮询 NOT_READY

```text
TX: FE 01 0D       # Window 1
TX: 0A 00 0D       # 读取GLOB_CMD
RX: 0A XX XX 0D
```

检查：

```text
GLOB_CMD bit10 = NOT_READY
```

必须满足：

```text
NOT_READY == 0
```

### 6.2 检查硬件错误

```text
TX: FE 00 0D       # Window 0
TX: 04 00 0D       # 读取DIAG_STAT
RX: 04 XX XX 0D
```

检查：

```text
DIAG_STAT bit[6:5] = HARD_ERR
```

正常值：

```text
HARD_ERR == 0
```

## 7. UART Manual Burst 模式

Manual Burst 由主机主动请求一整帧数据，适合 MCU 驱动和调试。

### 7.1 配置示例：16 bit、125 SPS

```text
FE 01 0D    # Window 1
85 04 0D    # SMPL_CTRL高字节=0x04，125 SPS
88 00 0D    # UART_AUTO=0，Manual Mode
8C 06 0D    # GPIO、COUNT输出
8D F0 0D    # FLAG、TEMP、GYRO、ACCL输出
8F 00 0D    # 16 bit输出
FE 00 0D    # Window 0
83 01 0D    # 进入Sampling Mode
```

### 7.2 每次读取

```text
TX: 80 00 0D
RX: 80 ...数据... 0D
```

推荐流程：

```text
等待DRDY有效
↓
发送 80 00 0D
↓
接收固定长度数据帧
↓
校验帧尾和Checksum
↓
解析数据
```

### 7.3 停止采样

```text
83 02 0D
```

返回 Configuration Mode 后才能安全修改大部分配置寄存器。

## 8. UART Auto Mode

Auto Mode 下，进入 Sampling Mode 后，IMU 按配置频率自动发送数据。

### 8.1 配置示例：16 bit、125 SPS

```text
FE 01 0D    # Window 1
85 04 0D    # 125 SPS
88 01 0D    # UART_AUTO=1
8C 06 0D    # GPIO、COUNT输出
8D F0 0D    # FLAG、TEMP、GYRO、ACCL输出
8F 00 0D    # 16 bit输出
FE 00 0D    # Window 0
83 01 0D    # 进入Sampling Mode
```

进入 Sampling Mode 后，无需继续发送读取命令。

### 8.2 Auto Mode限制

在以下状态下：

```text
UART Auto Mode + Sampling Mode
```

不要读取寄存器，否则寄存器响应会混入连续数据流，造成错帧。

需要修改配置时：

```text
83 02 0D    # 返回Configuration Mode
```

然后再执行寄存器读写。

## 9. 典型16 bit数据帧

以下配置：

```text
FLAG、TEMP、GYRO、ACCL、GPIO、COUNT输出
Checksum关闭
16 bit输出
```

对应固定22字节：

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 1 | `0x80` |
| 1 | 2 | FLAG |
| 3 | 2 | TEMP |
| 5 | 2 | XGYRO |
| 7 | 2 | YGYRO |
| 9 | 2 | ZGYRO |
| 11 | 2 | XACCL |
| 13 | 2 | YACCL |
| 15 | 2 | ZACCL |
| 17 | 2 | GPIO |
| 19 | 2 | COUNT |
| 21 | 1 | `0x0D` |

数据布局：

```text
80
FLAG_H FLAG_L
TEMP_H TEMP_L
XGYRO_H XGYRO_L
YGYRO_H YGYRO_L
ZGYRO_H ZGYRO_L
XACCL_H XACCL_L
YACCL_H YACCL_L
ZACCL_H ZACCL_L
GPIO_H GPIO_L
COUNT_H COUNT_L
0D
```

不要仅依靠搜索 `0x0D` 判断帧尾，因为传感器数据字段中也可能出现 `0x0D`。

推荐接收策略：

```text
1. 搜索帧头0x80
2. 按固定长度接收
3. 检查最后一个字节是否为0x0D
4. 可选检查Checksum
5. 解析数据
```

## 10. 数据解析

### 10.1 16 bit大端转换

```c
#include <stdint.h>

static int16_t imu_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}
```

解析示例：

```c
int16_t temp_raw = imu_be16(&frame[3]);

int16_t gx_raw = imu_be16(&frame[5]);
int16_t gy_raw = imu_be16(&frame[7]);
int16_t gz_raw = imu_be16(&frame[9]);

int16_t ax_raw = imu_be16(&frame[11]);
int16_t ay_raw = imu_be16(&frame[13]);
int16_t az_raw = imu_be16(&frame[15]);
```

### 10.2 单位换算

陀螺仪：

```c
float gx_dps = gx_raw * 0.016f;
float gy_dps = gy_raw * 0.016f;
float gz_dps = gz_raw * 0.016f;
```

单位：

```text
°/s
```

加速度计：

```c
float ax_g = ax_raw * 0.0002f;
float ay_g = ay_raw * 0.0002f;
float az_g = az_raw * 0.0002f;
```

单位：

```text
g
```

也可以换算为 `m/s²`：

```c
float ax_ms2 = ax_g * 9.80665f;
float ay_ms2 = ay_g * 9.80665f;
float az_ms2 = az_g * 9.80665f;
```

温度近似换算：

```c
float temperature_c =
    25.0f + ((float)temp_raw - 2634.0f) * -0.0037918f;
```

注意：温度字段主要用于内部温度补偿，不保证作为高精度绝对温度值。

## 11. 32 bit数据解析

32 bit 模式下，每个传感器字段由高16位和低16位组成。

```c
static int32_t imu_be32(const uint8_t *p)
{
    return (int32_t)(
        ((uint32_t)p[0] << 24) |
        ((uint32_t)p[1] << 16) |
        ((uint32_t)p[2] << 8)  |
        ((uint32_t)p[3])
    );
}
```

陀螺换算：

```c
float gyro_dps = raw32 * (0.016f / 65536.0f);
```

加速度换算：

```c
float accel_g = raw32 * (0.0002f / 65536.0f);
```

32 bit 只提高分辨率，不扩大：

```text
陀螺量程：±450 °/s
加速度量程：±5 g
```

## 12. Checksum

Checksum 可用于 UART Burst 和 UART Auto Mode。

计算规则：

```text
1. 从0x80之后的数据开始
2. 不包含最后的0x0D
3. 按16 bit无符号数累加
4. 仅保留低16 bit
```

示例代码：

```c
static uint16_t imu_checksum16(const uint16_t *data, uint16_t count)
{
    uint32_t sum = 0;

    for (uint16_t i = 0; i < count; ++i) {
        sum += data[i];
    }

    return (uint16_t)sum;
}
```

Checksum 不是 CRC，只是简单16 bit累加和。

## 13. DRDY使用

`Pin 13` 可配置为 DRDY。

典型流程：

```text
IMU完成一次采样
↓
DRDY有效
↓
主机发送Burst命令
↓
接收并解析一帧
```

注意：

- 如果启用了多个 ND 标志，但只读取部分字段，DRDY 可能无法正常撤销。
- Manual Burst 推荐结合 DRDY 中断使用。
- PC上位机通常不使用 DRDY，直接使用 Auto Mode 连续接收即可。

## 14. 串口带宽估算

UART 8N1 每发送1字节实际占10 bit。

460800 baud：

```text
460800 / 10 = 46080 Byte/s
```

16 bit、22字节帧：

```text
125 SPS  → 2750 Byte/s
1000 SPS → 22000 Byte/s
```

结论：

- 125 SPS、250 SPS、500 SPS 使用压力较小。
- 1000 SPS、16 bit 全字段可以使用 460800 baud。
- 1000 SPS、32 bit 全字段接近带宽上限。
- 230400 baud 不建议用于 1000 SPS 全字段 32 bit 输出。

## 15. 上位机使用建议

### 15.1 普通串口助手

可用于：

- 发送十六进制命令
- 验证寄存器读写
- 查看二进制原始帧

要求：

```text
十六进制发送
十六进制显示
不自动追加ASCII换行
命令末尾手动发送0D
```

### 15.2 自定义上位机

推荐功能：

- 固定长度帧同步
- 16/32 bit数据解析
- 单位换算
- Checksum校验
- 曲线显示
- CSV记录
- 丢帧统计
- COUNT连续性检查

### 15.3 官方评估工具

Epson 数据手册提到：

```text
USB I/F Board
Logger Software
```

可用于评估、采集和记录，但具体软件名称和获取方式需联系 Epson 或代理商。

## 16. 推荐调试步骤

```text
1. 使用460800、8N1打开串口
2. 上电后等待1 s
3. 发送 FE 00 0D
4. 读取 MODE_CTRL 或 DIAG_STAT
5. 确认收到以0D结尾的4字节响应
6. 配置Manual Burst、125 SPS、16 bit
7. 进入Sampling Mode
8. 周期发送 80 00 0D
9. 检查固定长度帧
10. 确认静止时：
    - 陀螺三轴接近0
    - 加速度模长接近1 g
```

## 17. 常见问题

### 17.1 完全无返回

检查：

- TX/RX 是否交叉连接
- 是否共地
- 是否使用3.3 V TTL
- 波特率是否为460800或230400
- 上电后是否等待800 ms
- UART和SPI是否同时连接
- `/RST` 是否被拉低

### 17.2 收到乱码

检查：

- 波特率
- 是否按二进制显示
- 是否错误地当作ASCII解析
- USB-UART是否稳定支持460800 baud

### 17.3 Auto Mode数据错位

原因通常包括：

- Auto Mode采样期间读取寄存器
- 按 `0x0D` 搜索帧尾而不是固定长度接收
- Burst字段配置与解析结构不一致
- 16/32 bit配置不一致
- 串口带宽不足造成丢字节

### 17.4 静止时数据不为零

正常现象：

- 陀螺存在零偏
- 加速度计会测得重力
- 安装倾斜会使重力分量分布到多个轴
- 刚进入Sampling Mode时，数字滤波器可能处于瞬态阶段

建议丢弃启动后的前若干帧。

## 18. 最小可用命令序列

### Manual Burst，16 bit，125 SPS

```text
FE 01 0D
85 04 0D
88 00 0D
8C 06 0D
8D F0 0D
8F 00 0D
FE 00 0D
83 01 0D
```

读取一帧：

```text
80 00 0D
```

停止采样：

```text
83 02 0D
```

### Auto Mode，16 bit，125 SPS

```text
FE 01 0D
85 04 0D
88 01 0D
8C 06 0D
8D F0 0D
8F 00 0D
FE 00 0D
83 01 0D
```

停止自动输出：

```text
83 02 0D
```
