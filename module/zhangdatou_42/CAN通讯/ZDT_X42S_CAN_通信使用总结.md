# ZDT_X42S CAN 通讯使用总结

> 适用对象：ZDT_X42S 第二代闭环步进电机，STM32F407 + bxCAN  
> 参考手册：`ZDT_X42S 第二代闭环步进电机用户手册 V1.0.5`

## 1. 协议概述

ZDT_X42S 的 CAN 通讯采用：

```text
Classic CAN 2.0
扩展帧 Extended Frame
数据帧 Data Frame
单帧最大数据长度 8 Byte
厂家自定义协议
```

核心区别：

```text
串口协议：电机地址位于数据首字节
CAN 协议：电机地址编码在扩展帧 ID 中
```

扩展帧 ID 计算：

```c
ExtId = ((uint32_t)motor_addr << 8) | packet_index;
```

字段含义：

```text
motor_addr   电机地址，1~255
packet_index 分包编号，从 0 开始
地址 0       广播地址
```

示例：

| 电机地址 | 分包编号 | 扩展帧 ID |
|---:|---:|---:|
| 1 | 0 | `0x0100` |
| 1 | 1 | `0x0101` |
| 2 | 0 | `0x0200` |
| 0（广播） | 0 | `0x0000` |

注意：

- 必须使用扩展帧，不能使用标准帧。
- CAN 数据区中不再包含电机地址。
- 默认校验字节为固定 `0x6B`。
- CANopen 是另一套协议，不能与本手册中的自由协议混用。

## 2. 电机端参数配置

在电机菜单中设置：

```text
P_Serial = CAN1_MAP
CAN_Baud = 500000
ID_Addr  = 1
Checksum = 0x6B
Response = Receive
```

### 2.1 CAN 波特率

支持：

```text
10000
20000
50000
83333
100000
125000
250000
500000
800000
1000000 bit/s
```

默认：

```text
500000 bit/s
```

所有节点必须使用相同波特率。

### 2.2 电机地址

```text
1~255：单节点地址
0：广播地址
```

多机总线中每台电机必须设置不同地址。

### 2.3 校验方式

可配置：

```text
0x6B
XOR
CRC-8
```

建议调试阶段使用：

```text
Checksum = 0x6B
```

### 2.4 Response 应答模式

| 模式 | 行为 |
|---|---|
| `None` | 不返回接收确认，也不返回动作完成 |
| `Receive` | 收到正确命令后立即确认，默认 |
| `Reached` | 仅返回到位、回零完成等动作完成消息 |
| `Both` | 同时返回接收确认和动作完成 |
| `Other` | 位置到位返回，其他动作返回接收确认 |

推荐：

```text
调试阶段：Receive
需要判断动作完成：Both
```

## 3. 硬件接线

电机 CAN 引脚定义：

```text
R/A/H -> CAN_H
T/B/L -> CAN_L
GND   -> GND
```

STM32 的 CAN_TX/CAN_RX 不能直接接 CANH/CANL，必须经过 CAN 收发器。

典型连接：

```text
STM32 CAN_TX -> CAN收发器 TXD
STM32 CAN_RX <- CAN收发器 RXD

CAN收发器 CANH -> 电机 R/A/H
CAN收发器 CANL -> 电机 T/B/L
GND             -> 电机 GND
```

STM32F407 常用引脚：

```text
PB9 -> CAN1_TX
PB8 -> CAN1_RX
```

总线要求：

```text
线形拓扑
总线两端各接 120Ω
所有节点共地
支线尽量短
CANH/CANL 不可接反
```

断电测量 CANH 与 CANL：

```text
正常约 60Ω
```

这是两个 `120Ω` 终端电阻并联的结果。

## 4. STM32F407 CubeMX 配置

已知：

```text
PCLK1 = 42 MHz
CAN1 Clock = 42 MHz
```

配置 500 kbit/s：

```text
Operating Mode                    Normal
Prescaler                         6
Time Quanta in Bit Segment 1      11 TQ
Time Quanta in Bit Segment 2      2 TQ
ReSynchronization Jump Width      1 TQ
Automatic Bus-Off Management      Enable
Automatic Retransmission          Enable
Automatic Wake-Up                 Disable
Receive FIFO Locked Mode          Disable
Transmit FIFO Priority            Disable
```

波特率计算：

```text
42 MHz / [6 × (1 + 11 + 2)]
= 500 kbit/s
```

NVIC：

```text
CAN1 RX0 interrupt = Enable
```

GPIO：

```text
PB8 -> CAN1_RX
PB9 -> CAN1_TX
Alternate Function = AF9_CAN1
Pull = No Pull
Speed = Very High
```

调试初期，过滤器建议先配置为全部接收。

## 5. STM32 HAL 发送头配置

```c
CAN_TxHeaderTypeDef tx_header = {0};

tx_header.IDE = CAN_ID_EXT;
tx_header.RTR = CAN_RTR_DATA;
tx_header.ExtId = ((uint32_t)motor_id << 8) | packet_index;
tx_header.DLC = data_length;
tx_header.TransmitGlobalTime = DISABLE;
```

错误配置：

```c
tx_header.IDE = CAN_ID_STD;
```

使用标准帧时，电机无法识别。

## 6. 单帧命令格式

对于不超过 8 字节的数据，直接使用一个 CAN 帧发送。

通用形式：

```text
ExtID = (Addr << 8) | 0
Data  = 功能码 + 命令数据 + 校验码
```

### 6.1 读取实时位置

原串口命令：

```text
01 36 6B
```

转换为 CAN：

```text
ExtID = 0x0100
DLC   = 2
Data  = 36 6B
```

### 6.2 读取实时转速

```text
ExtID = 0x0100
DLC   = 2
Data  = 35 6B
```

### 6.3 使能电机

原串口命令：

```text
01 F3 AB 01 00 6B
```

CAN：

```text
ExtID = 0x0100
DLC   = 5
Data  = F3 AB 01 00 6B
```

参数：

```text
F3：使能控制功能码
AB：辅助码
01：使能
00：立即执行
6B：校验码
```

失能：

```text
Data = F3 AB 00 00 6B
```

### 6.4 立即停止

```text
ExtID = 0x0100
DLC   = 4
Data  = FE 98 00 6B
```

## 7. 长命令分包规则

命令数据超过 8 字节时，需要拆成多个 CAN 帧。

规则：

```text
Packet 从 0 开始递增
每包第一个字节都必须重复功能码
第一包最多携带 功能码 + 7 字节参数
后续包最多携带 功能码 + 7 字节后续数据
最后一包包含校验码
同一命令的分包必须连续发送
分包间不要插入其他命令
```

### 7.1 Emm 位置模式示例

原串口命令：

```text
01 FD 01 0F A0 00 00 01 FA 00 00 00 6B
```

CAN 分包：

```text
第 0 包：
ExtID = 0x0100
DLC   = 8
Data  = FD 01 0F A0 00 00 01 FA

第 1 包：
ExtID = 0x0101
DLC   = 5
Data  = FD 00 00 00 6B
```

错误写法：

```text
第二包 Data = 00 00 00 6B
```

正确写法：

```text
第二包 Data = FD 00 00 00 6B
```

每个分包首字节必须重复功能码 `FD`。

## 8. 电机返回帧

返回帧同样使用：

```text
ExtID = (Addr << 8) | Packet
```

数据区：

```text
功能码 + 返回数据 + 校验码
```

常见返回码：

| 返回码 | 含义 |
|---:|---|
| `02` | 命令接收正确 |
| `12` / `22` | 当前已满足回零或限位条件，电机不运动 |
| `E2` | 参数错误或当前状态不允许执行 |
| `EE` | 命令格式错误 |
| `9F` | 动作执行完成 |

示例：

```text
使能成功：
Data = F3 02 6B

位置命令接收成功：
Data = FD 02 6B

位置运动完成：
Data = FD 9F 6B

回零完成：
Data = 9A 9F 6B
```

注意：

```text
02 只表示命令已接收
9F 才表示动作已完成
```

## 9. 常用功能码

### 9.1 通用控制

| 功能码 | 功能 |
|---:|---|
| `06` | 编码器校准 |
| `08` | 重启电机 |
| `0A` | 当前角度清零 |
| `0E` | 解除堵转/过热/过流保护 |
| `0F` | 恢复出厂设置 |
| `F3` | 电机使能/失能 |
| `FE` | 立即停止 |
| `FF` | 触发多机同步运动 |
| `9A` | 触发回零 |
| `9C` | 强制中断回零 |

### 9.2 状态读取

| 功能码 | 功能 |
|---:|---|
| `1F` | 读取固件和硬件版本 |
| `24` | 读取总线电压 |
| `26` | 读取总线电流 |
| `27` | 读取相电流 |
| `31` | 读取编码器值 |
| `35` | 读取实时转速 |
| `36` | 读取实时位置 |
| `37` | 读取位置误差 |
| `39` | 读取驱动温度 |
| `3A` | 读取电机状态标志 |
| `3B` | 读取回零状态标志 |

### 9.3 Emm 固件运动命令

| 功能码 | 功能 |
|---:|---|
| `F6` | 速度模式 |
| `FD` | 位置模式 |
| `F1` | 快速位置模式参数配置 |
| `FC` | 快速更新位置 |

### 9.4 X 固件运动命令

| 功能码 | 功能 |
|---:|---|
| `F5` | 力矩模式 |
| `C5` | 力矩模式限速 |
| `F6` | 速度模式 |
| `C6` | 速度模式限电流 |
| `FB` | 直通位置模式 |
| `CB` | 直通位置模式限电流 |
| `FD` | 梯形位置模式 |
| `CD` | 梯形位置模式限电流 |
| `F1` | 快速位置参数配置 |
| `FC` | 快速更新位置 |

注意：

- Emm 和 X 固件部分命令使用相同功能码，但数据结构不同。
- 程序必须明确当前电机使用的固件类型。

## 10. 多电机通信

每台电机设置不同地址：

```text
电机 1：ID = 1
电机 2：ID = 2
电机 3：ID = 3
```

发送目标：

```text
0x0100 -> 电机 1
0x0200 -> 电机 2
0x0300 -> 电机 3
```

## 11. 广播命令

广播地址：

```text
Addr = 0
ExtID = 0x0000
```

适用于所有电机执行同一条命令。

广播时应注意多个节点同时回复可能导致总线冲突，因此：

- 根据手册规定使用广播命令。
- 必要时将 `Response` 设置为 `None` 或 `Reached`。
- 不要让多台电机同时回复普通查询命令。

## 12. 多机同步运动

不同电机执行不同命令并同步启动：

1. 分别向每台电机发送运动命令。
2. 将运动命令中的同步标志设为 `01`。
3. 电机仅缓存命令，不立即执行。
4. 最后广播触发同步运动。

同步触发串口形式：

```text
00 FF 66 6B
```

CAN 形式：

```text
ExtID = 0x0000
DLC   = 3
Data  = FF 66 6B
```

执行结果：

```text
已缓存命令的电机同时启动
未缓存命令的电机不动作
```

## 13. 多电机命令 0xAA

X42S 支持把多个电机命令组合成一条逻辑命令。

逻辑格式：

```text
00 AA 总字节数 电机1命令 电机2命令 ... 6B
```

由于 CAN 单帧最多 8 字节，需要分包。

分包规则：

```text
ExtID = 0x0000、0x0001、0x0002 ...
每包首字节都重复 AA
```

注意：

- 多电机命令需要连续发送。
- 不要在分包间插入其他命令。
- 运动命令通常只允许指定电机回复，避免总线冲突。
- 多电机命令中包含读取命令时，需要设计返回时序。

## 14. 定时返回信息

功能码：

```text
11
```

用途：

```text
周期返回实时位置
周期返回实时转速
周期返回电流
周期返回状态标志
```

示例：电机 2 每 10 ms 返回状态标志。

原命令：

```text
02 11 18 3A 00 0A 6B
```

CAN：

```text
ExtID = 0x0200
DLC   = 6
Data  = 11 18 3A 00 0A 6B
```

停止定时返回：

```text
定时时间 = 0
```

示例：

```text
ExtID = 0x0200
Data  = 11 18 3A 00 00 6B
```

注意：

- 多台电机不建议都设置为 1 ms 周期返回。
- 应根据总线负载合理设置周期。
- 推荐位置、速度反馈周期从 10~20 ms 开始测试。

## 15. 心跳保护

心跳保护用于：

```text
在设定时间内未收到有效命令时自动急停
```

适合：

```text
小车
机械臂
实时速度控制
实时位置控制
```

正式运行建议开启，避免 CAN 通讯中断后电机继续运动。

## 16. STM32 接收处理建议

接收时先判断：

```c
if (rx_header.IDE != CAN_ID_EXT) {
    return;
}
```

提取地址和分包号：

```c
uint8_t motor_addr = (uint8_t)((rx_header.ExtId >> 8) & 0xFFU);
uint8_t packet_id  = (uint8_t)(rx_header.ExtId & 0xFFU);
```

进一步判断：

```text
Data[0]：功能码
Data[1]：返回码或返回数据首字节
最后一字节：校验码
```

中断回调中建议只做：

```text
读取报文
复制数据
记录时间戳
放入环形缓冲区或消息队列
```

不要在中断中执行：

```text
printf
阻塞式串口发送
长时间计算
复杂状态机
```

## 17. 推荐调试顺序

```text
1. 电机 P_Serial 设置为 CAN1_MAP
2. 电机 CAN_Baud 设置为 500000
3. 电机 ID 设置为 1
4. Checksum 使用默认 0x6B
5. Response 设置为 Receive
6. STM32 CAN1 配置 500 kbit/s
7. 过滤器先全部接收
8. 发送读取实时位置
9. 确认收到 36 开头的返回帧
10. 发送读取实时转速
11. 发送电机使能命令
12. 测试低速速度模式
13. 测试立即停止
14. 测试位置模式
15. 最后测试长命令分包和多机同步
```

## 18. 最小测试报文

### 18.1 读取实时位置

```text
ExtID = 0x0100
DLC   = 2
Data  = 36 6B
```

### 18.2 读取实时转速

```text
ExtID = 0x0100
DLC   = 2
Data  = 35 6B
```

### 18.3 使能电机

```text
ExtID = 0x0100
DLC   = 5
Data  = F3 AB 01 00 6B
```

### 18.4 失能电机

```text
ExtID = 0x0100
DLC   = 5
Data  = F3 AB 00 00 6B
```

### 18.5 立即停止

```text
ExtID = 0x0100
DLC   = 4
Data  = FE 98 00 6B
```

## 19. 常见错误

### 19.1 使用标准帧

错误：

```text
IDE = Standard
```

正确：

```text
IDE = Extended
```

### 19.2 数据区仍包含地址

错误：

```text
ExtID = 0x0100
Data  = 01 36 6B
```

正确：

```text
ExtID = 0x0100
Data  = 36 6B
```

### 19.3 长命令第二包未重复功能码

错误：

```text
ExtID = 0x0101
Data  = 00 00 00 6B
```

正确：

```text
ExtID = 0x0101
Data  = FD 00 00 00 6B
```

### 19.4 把 02 当作动作完成

```text
02：命令接收成功
9F：动作执行完成
```

### 19.5 没有 CAN 收发器

STM32 的 CAN_TX/CAN_RX 不能直接连接电机 CANH/CANL。

### 19.6 没有终端电阻

CAN 总线两端必须各接一个 `120Ω`。

### 19.7 波特率不一致

STM32、电机、USB-CAN 适配器必须使用相同波特率。

## 20. 核心结论

```text
1. 使用 Classic CAN 扩展帧
2. ExtID = 电机地址 << 8 | 分包编号
3. CAN 数据中不包含电机地址
4. 默认波特率为 500 kbit/s
5. 默认校验字节为 0x6B
6. 地址 0 为广播
7. 长命令每包首字节都重复功能码
8. 02 表示接收成功，9F 表示动作完成
9. STM32 必须通过 CAN 收发器连接 CANH/CANL
10. 多机同步通过同步标志 + 广播 FF 66 6B 实现
```
