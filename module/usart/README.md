# HC-12 串口模块

本目录保存天猛星 MSPM0G3507 与 HC-12 透明串口无线模块相关的配置和驱动。
HC-12 是串口透明传输无线模块，不是蓝牙协议模块。

## 文件

- `HC-12.pdf`：模块资料。
- `hc12_config.h`：软件缓冲区和中断优先级配置。
- `hc12_uart.h/.c`：初始化、发送、非阻塞接收和接收中断实现。

## 引脚与串口配置

`DianSai_MSPM0G3507.syscfg` 中新增了 `UART_HC12`：

| 配置项 | 值 |
|---|---|
| MSPM0 外设 | UART3 |
| TX | PB12 |
| RX | PB13 |
| 波特率 | 9600 |
| 数据格式 | 8 位数据、无校验、1 位停止位（8-N-1） |
| FIFO | 开启，RX 阈值为 1 字节 |
| 接收方式 | UART RX 中断 |
| 硬件流控 | 不使用 |

接线时 TX/RX 需要交叉：

| HC-12 | 天猛星 MSPM0G3507 |
|---|---|
| RXD | PB12 / UART3_TX |
| TXD | PB13 / UART3_RX |
| GND | GND |
| VCC | 按手中模块规格连接 |

扩展板上的 `RX/B12` 表示模块 RXD 接 PB12，`TX/B13` 表示模块 TXD
接 PB13。

当前没有为 HC-12 的 `SET` 分配 GPIO。透明传输不需要控制 `SET`；如果后续
需要通过程序进入 AT 模式，应先选择一个空闲 GPIO，再在 SysConfig 中添加。

## 基本使用

系统启动时必须先执行工程原有的 `SYSCFG_DL_init()`，然后初始化 HC-12：

```c
#include "hc12_uart.h"

void AppInit(void)
{
    Hc12Init();
    (void)Hc12WriteString("HC12 ready\r\n");
}
```

非阻塞读取：

```c
uint8_t buffer[64];
size_t received;

received = Hc12Read(buffer, sizeof(buffer));
if (received > 0U)
{
    /* 处理 buffer[0..received-1] */
}
```

发送二进制数据：

```c
static const uint8_t message[] = {0xAAU, 0x55U, 0x01U};

(void)Hc12Write(message, sizeof(message));
```

## 接收缓冲区

UART3 中断服务函数已经在 `hc12_uart.c` 中实现，不要在其他文件中重复定义
`UART3_IRQHandler` 或 `UART_HC12_INST_IRQHandler`。

接收数据先进入 `HC12_RX_BUFFER_SIZE` 字节的软件环形缓冲区。默认容量为
256 字节，可以在 `hc12_config.h` 中修改。缓冲区满时保留最新数据并丢弃
最旧字节，可用以下接口读取并清零溢出次数：

```c
uint32_t overflow_count = Hc12GetAndClearOverflowCount();
```

## 注意事项

1. HC-12 两端必须使用相同的波特率、信道和工作模式。
2. 当前 UART 波特率由 SysConfig 生成；修改波特率时同时更新
   `hc12_config.h` 中的说明值。
3. `Hc12Write` 是阻塞发送接口，不要在中断服务函数中调用。
4. `Hc12Read` 是非阻塞接口，可以在 FreeRTOS 任务中周期调用。
5. 本模块不调用 FreeRTOS API，因此 UART3 中断不受内核 ISR API 约束。
