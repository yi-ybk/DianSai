# HC-12 UART3 FreeRTOS 使用说明

本文说明天猛星 MSPM0G3507 开发板通过 UART3 使用 HC-12 透明串口模块时，
如何在 FreeRTOS 多任务工程中安全地进行发送和接收。

## 1. 文件结构

| 文件 | 作用 |
|---|---|
| `hc12_config.h` | 波特率、软件接收缓冲区和 UART3 中断优先级 |
| `hc12_uart.h/.c` | 不依赖操作系统的底层 UART3 驱动 |
| `hc12_freertos.h/.c` | FreeRTOS 任务安全封装 |
| `README.md` | 硬件接线与底层接口说明 |
| `FREERTOS.md` | FreeRTOS 接口、任务模型和示例 |

硬件配置仍由工程根目录的 `DianSai_MSPM0G3507.syscfg` 生成：

| 配置 | 值 |
|---|---|
| 外设 | UART3 |
| TX | PB12，连接 HC-12 RXD |
| RX | PB13，连接 HC-12 TXD |
| 串口格式 | 9600、8-N-1 |
| 接收方式 | UART RX 中断 |
| UART FIFO | 开启，RX 阈值 1 字节 |

## 2. 为什么需要 FreeRTOS 封装

底层 `Hc12Write()` 是阻塞发送。如果两个任务同时调用，两个任务的数据可能
交织成一条错误消息。底层 `Hc12Read()` 是非阻塞读取，也不能让任务等待新数据。

FreeRTOS 封装提供：

- 静态 TX 互斥量：保证一条消息完整发送；
- 静态 RX 互斥量：保证读取固定长度帧时不会被其他任务抢走中间字节；
- 静态二值信号量：UART3 收到数据后由 ISR 唤醒接收任务；
- 带 Tick 超时的读取；
- 发送、接收、超时和缓冲区溢出统计；
- 全部同步对象使用静态内存，不占用 FreeRTOS 堆。

## 3. 初始化

在 `SYSCFG_DL_init()` 之后调用一次 `Hc12RtosInit()`。推荐在调度器启动前
初始化同步对象和 UART3：

```c
#include <FreeRTOS.h>
#include <task.h>

#include "hc12_freertos.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();

    configASSERT(Hc12RtosInit());

    /* 在这里创建应用任务。 */
    robotInit();
    vTaskStartScheduler();

    for (;;)
        ;
}
```

`Hc12RtosInit()` 只调用一次。调度器启动之前若需要发送启动调试信息，可直接
使用底层 `Hc12WriteString()`；`Hc12RtosWriteString()` 等任务接口应在调度器
启动后调用。

## 4. 基本接口

| 接口 | 作用 |
|---|---|
| `Hc12RtosWrite` | 线程安全发送二进制数据 |
| `Hc12RtosWriteString` | 线程安全发送字符串 |
| `Hc12RtosRead` | 等待至少一个字节，返回当前已有数据 |
| `Hc12RtosReadExact` | 等待指定长度，适合固定长度协议帧 |
| `Hc12RtosReadByte` | 带超时读取一个字节 |
| `Hc12RtosAvailable` | 获取缓冲区待读字节数快照 |
| `Hc12RtosFlushRx` | 线程安全清空接收缓冲区 |
| `Hc12RtosGetStats` | 获取收发、超时和丢字节统计 |
| `Hc12RtosResetStats` | 清零统计 |

公共状态码：

| 状态 | 含义 |
|---|---|
| `HC12_RTOS_OK` | 操作成功 |
| `HC12_RTOS_TIMEOUT` | 等待互斥量或接收数据超时 |
| `HC12_RTOS_NOT_INITIALIZED` | 尚未调用 `Hc12RtosInit()` |
| `HC12_RTOS_INVALID_ARGUMENT` | 空指针、零长度等非法参数 |
| `HC12_RTOS_SCHEDULER_NOT_RUNNING` | 在调度器运行前调用了任务接口 |
| `HC12_RTOS_DRIVER_ERROR` | 底层 UART 驱动返回失败 |

## 5. 发送任务示例

```c
static void Hc12TxTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        Hc12RtosStatus status;

        status = Hc12RtosWriteString(
            "MSPM0 online\r\n",
            pdMS_TO_TICKS(100U));

        if (status != HC12_RTOS_OK)
        {
            /* 记录或处理错误。 */
        }

        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
```

发送接口的超时参数只限制等待 TX 互斥量的时间。取得互斥量后，底层 UART
会阻塞发送到全部字节写入完成。9600 波特率下发送一个 10 位串口字符约需
1.04 ms，因此不要在高实时性任务中一次发送过大的数据块。

## 6. 接收任务示例

读取当前一批数据：

```c
static void Hc12RxTask(void *argument)
{
    uint8_t buffer[64];

    (void)argument;

    for (;;)
    {
        size_t received = 0U;
        Hc12RtosStatus status;

        status = Hc12RtosRead(
            buffer,
            sizeof(buffer),
            &received,
            pdMS_TO_TICKS(200U));

        if (status == HC12_RTOS_OK)
        {
            /* 处理 buffer[0..received-1]。 */
        }
        else if (status == HC12_RTOS_TIMEOUT)
        {
            /* 200 ms 内没有收到数据。 */
        }
    }
}
```

读取固定 8 字节协议帧：

```c
uint8_t frame[8];
size_t received = 0U;

Hc12RtosStatus status = Hc12RtosReadExact(
    frame,
    sizeof(frame),
    &received,
    pdMS_TO_TICKS(100U));

if (status == HC12_RTOS_OK)
{
    /* 收到完整的 8 字节。 */
}
else if ((status == HC12_RTOS_TIMEOUT) && (received > 0U))
{
    /* 超时，但 received 表示已收到的部分长度。 */
}
```

传入 `portMAX_DELAY` 可以一直等待。

## 7. 推荐任务模型

串口是连续字节流。虽然 RX 互斥量允许多个任务调用接收接口，但多个任务会
竞争同一份数据，无法知道哪个任务会取走下一段字节。

推荐只创建一个 HC-12 接收任务：

1. 接收任务调用 `Hc12RtosRead()`；
2. 接收任务完成帧头、长度、校验和解析；
3. 将完整协议消息通过 FreeRTOS Queue 分发给控制任务；
4. 其他任务只使用发送接口，不直接读取 UART3。

初始化 FreeRTOS 封装后，不要在其他任务中混用底层 `Hc12Read()`，否则会
绕过 RX 互斥量并取走接收任务的数据。多任务发送也不要混用底层
`Hc12Write()`。

## 8. 统计与诊断

```c
Hc12RtosStats stats;

if (Hc12RtosGetStats(&stats))
{
    /* stats.tx_bytes：成功发送字节数 */
    /* stats.rx_bytes：任务接口读取字节数 */
    /* stats.read_timeouts：接收超时次数 */
    /* stats.rx_dropped_bytes：环形缓冲区满后丢弃的旧字节数 */
}

(void)Hc12RtosFlushRx(pdMS_TO_TICKS(20U));
Hc12RtosResetStats();
```

如果 `rx_dropped_bytes` 持续增加，可增大 `HC12_RX_BUFFER_SIZE`、提高接收
任务优先级，或缩短接收任务处理数据的时间。

## 9. 中断与 FreeRTOS 注意事项

- `UART3_IRQHandler` 已由 `hc12_uart.c` 实现，不要重复定义；
- UART3 中断只搬运数据并发出 FromISR 通知，不在 ISR 内解析协议；
- UART3 中断优先级为 2；
- 当前 FreeRTOS 的 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 为 1，
  因而优先级 2 可以调用 `xSemaphoreGiveFromISR()`；
- `hc12_uart.c` 不依赖 FreeRTOS，`hc12_freertos.c` 才依赖 FreeRTOS；
- HC-12 是透明串口无线模块，不是蓝牙协议模块。
