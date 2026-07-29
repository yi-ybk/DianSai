# NRF24L01+ FreeRTOS 任务调用说明

本文说明如何在天猛星 MSPM0G3507 的 FreeRTOS 工程中，通过
`nrf24l01_freertos.h/.c` 安全地让多个任务共享同一个 NRF24L01+。

## 1. 驱动分层

| 文件 | 作用 |
|---|---|
| `nrf24l01_port_mspm0.h/.c` | MSPM0 GPIO、模拟 SPI 和延时适配 |
| `nrf24l01.h/.c` | NRF24L01+ 寄存器和收发底层驱动 |
| `nrf24l01_simple.h/.c` | 单线程应用便捷封装 |
| `nrf24l01_freertos.h/.c` | FreeRTOS 任务安全封装 |
| `nrf24l01.md` | 接线、无线参数和底层接口说明 |
| `nrf24l01_freertos.md` | FreeRTOS 初始化、任务模型和接口示例 |

业务任务应优先使用 `Nrf24Rtos*` 接口。FreeRTOS 封装初始化完成后，不要在
其他任务中直接调用同一实例的 `Nrf24Simple*` 或 `Nrf24*` 接口，否则会绕过
互斥量。

## 2. 默认硬件与无线参数

`Nrf24RtosGetTianmengxingConfig()` 复用已经与 MaixCAM 通信验证过的配置：

| 配置 | 值 |
|---|---|
| SCK/MOSI/MISO | PB23/PB22/PB21，GPIO 模拟 SPI |
| CE/CSN/IRQ | PB24/PB26/PB27 |
| 地址 | `D2 F0 F0 F0 A5` |
| 频道 | 76 |
| 速率 | 1 Mbps |
| 发射功率 | -18 dBm |
| CRC | 2 字节 |
| 自动应答 | 开启 |
| 固定负载 | 32 字节 |
| 自动重发 | 15 次，间隔 1 ms |

MaixCAM 和 MSPM0 两端的频道、地址、速率、CRC、自动应答和负载模式必须
一致。

## 3. FreeRTOS 同步模型

每个 `Nrf24Rtos_t` 内部包含两个静态互斥量：

- `device_mutex`：保护 CE、CSN、SPI、寄存器和收发模式，确保一次完整驱动
  调用不会与其他任务交织；
- `receive_mutex`：保证只有一个任务消费 RX FIFO，避免一包数据被错误任务
  取走。

互斥量使用 `xSemaphoreCreateMutexStatic()`，不会占用
`configTOTAL_HEAP_SIZE`。

PB27 当前配置为普通输入，工程现有 `GROUP1_IRQHandler` 尚未分发 NRF24
IRQ。因此接收接口使用 FreeRTOS 友好的轮询：

1. 取得设备互斥量；
2. 非阻塞检查一次 RX FIFO；
3. 释放设备互斥量；
4. 没有数据时调用 `vTaskDelay(1)`，让出 CPU；
5. 超时前重复检查。

等待期间不会长时间占有设备互斥量，发送任务仍能访问模块。这不是忙等。

## 4. 初始化

无线对象应定义在全局或静态作用域，初始化后不能复制或移动：

```c
#include <FreeRTOS.h>
#include <task.h>

#include "nrf24l01_freertos.h"
#include "ti_msp_dl_config.h"

static Nrf24Rtos_t g_radio;

static void RadioInit(void)
{
    Nrf24InitConfig_t config;
    Nrf24Result_t result;

    Nrf24RtosGetTianmengxingConfig(&config);
    result = Nrf24RtosInit(&g_radio, &config, true);
    configASSERT(result == NRF24_RESULT_OK);
}

int main(void)
{
    SYSCFG_DL_init();
    RadioInit();

    /* 创建应用任务。 */
    robotInit();
    vTaskStartScheduler();

    for (;;)
        ;
}
```

`Nrf24RtosInit()` 包含约 100 ms 的阻塞上电等待，推荐在启动调度器前调用一次。
其他 `Nrf24Rtos*` 接口必须在调度器运行后调用。

## 5. 发送任务

```c
static void RadioTxTask(void *argument)
{
    static const uint8_t message[] = "hello";

    (void)argument;
    for (;;)
    {
        Nrf24Result_t result;

        result = Nrf24RtosSend(
            &g_radio,
            message,
            (uint8_t)(sizeof(message) - 1U),
            150U,                  /* 无线发送/ACK 超时，ms */
            pdMS_TO_TICKS(100U),   /* 等待设备互斥量 */
            true);                 /* 发送后恢复 RX */

        if (result == NRF24_RESULT_OK)
        {
            /* 对端 NRF24 已返回硬件 ACK。 */
        }
        else if (result == NRF24_RESULT_MAX_RETRIES)
        {
            /* 自动重发耗尽，检查对端、频道、地址和供电。 */
        }
        else if (result == NRF24_RESULT_LOCK_TIMEOUT)
        {
            /* 其他任务占用模块时间过长。 */
        }

        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
```

`radio_timeout_ms` 是无线发送和硬件 ACK 的超时；`lock_timeout_ticks` 是
等待设备互斥量的超时，两者含义不同。

固定 32 字节负载模式下，少于 32 字节的数据会由便捷层在尾部补零。

## 6. 接收任务

推荐只有一个任务负责接收并解析协议：

```c
static void RadioRxTask(void *argument)
{
    uint8_t payload[NRF24_MAX_PAYLOAD_SIZE];

    (void)argument;
    for (;;)
    {
        uint8_t length = 0U;
        uint8_t pipe = 0xFFU;
        Nrf24Result_t result;

        result = Nrf24RtosReceive(
            &g_radio,
            payload,
            (uint8_t)sizeof(payload),
            &length,
            &pipe,
            pdMS_TO_TICKS(500U));

        if (result == NRF24_RESULT_OK)
        {
            /* 处理 payload[0..length-1]。 */
        }
        else if (result == NRF24_RESULT_TIMEOUT)
        {
            /* 500 ms 内没有收到无线数据。 */
        }
        else if (result == NRF24_RESULT_LOCK_TIMEOUT)
        {
            /* 无法在剩余超时内取得互斥量。 */
        }
    }
}
```

超时参数规则：

- `0`：只进行一次非阻塞检查，无数据返回 `NRF24_RESULT_NO_DATA`；
- 正 Tick 数：最多等待对应时间，无数据返回 `NRF24_RESULT_TIMEOUT`；
- `portMAX_DELAY`：一直等待。

固定负载模式返回长度为 32。应用协议应在负载中设置自己的有效长度字段。

接收任务完成帧头、长度和校验解析后，可以通过 FreeRTOS Queue 将完整消息
分发给控制任务，其他任务不应直接竞争 RX FIFO。

## 7. 模式和状态接口

```c
Nrf24Result_t result;
bool available;
bool connected;

result = Nrf24RtosStartReceive(
    &g_radio, false, pdMS_TO_TICKS(20U));

result = Nrf24RtosDataAvailable(
    &g_radio, &available, pdMS_TO_TICKS(20U));

result = Nrf24RtosCheckConnection(
    &g_radio, &connected, pdMS_TO_TICKS(20U));

result = Nrf24RtosStopReceive(
    &g_radio, pdMS_TO_TICKS(20U));

result = Nrf24RtosPowerDown(
    &g_radio, pdMS_TO_TICKS(20U));
```

`Nrf24RtosStartReceive(..., true, ...)` 会清空尚未读取的旧数据，只有确认不需要
旧包时才使用 `true`。

## 8. 返回值

除原有 NRF24 驱动错误外，FreeRTOS 封装增加了两个返回值：

| 返回值 | 含义 |
|---|---|
| `NRF24_RESULT_RTOS_NOT_RUNNING` | 调度器尚未运行或当前被挂起 |
| `NRF24_RESULT_LOCK_TIMEOUT` | 等待设备或接收互斥量超时 |

常见原有返回值：

| 返回值 | 含义 |
|---|---|
| `NRF24_RESULT_OK` | 操作成功 |
| `NRF24_RESULT_NO_DATA` | 非阻塞检查时没有数据 |
| `NRF24_RESULT_TIMEOUT` | 接收等待或无线发送超时 |
| `NRF24_RESULT_MAX_RETRIES` | 自动重发耗尽 |
| `NRF24_RESULT_BUFFER_TOO_SMALL` | 接收缓冲区小于无线负载 |
| `NRF24_RESULT_DEVICE_NOT_FOUND` | SPI 回读不符合预期 |

## 9. 统计

```c
Nrf24RtosStats_t stats;

if (Nrf24RtosGetStats(&g_radio, &stats))
{
    /* stats.tx_packets、stats.tx_bytes */
    /* stats.tx_failures */
    /* stats.rx_packets、stats.rx_bytes */
    /* stats.receive_timeouts */
    /* stats.lock_timeouts */
}

Nrf24RtosResetStats(&g_radio);
```

`tx_failures` 统计已经取得设备互斥量、但无线发送失败的次数；互斥量超时单独
计入 `lock_timeouts`。

## 10. 使用限制与注意事项

1. `Nrf24RtosInit()` 只调用一次，对象必须保持在静态存储区。
2. 初始化后不要混用同一实例的裸驱动或 `Nrf24Simple*` 接口。
3. 多个发送任务可以安全调用 `Nrf24RtosSend()`。
4. 虽然接收互斥量能串行化多个接收任务，仍推荐只设置一个接收任务。
5. 软件 SPI 不会禁止任务调度；其他任务可能拉长时钟边沿间隔，但不会破坏
   本模块的互斥访问。
6. NRF24L01+ 只能使用 3.3V，模块旁应增加 100 nF 和 10–47 µF 去耦电容。
7. 当前 `test.c` 和 `Nrf24Stage1TestRun()` 是调度器启动前的裸机测试，不应
   与 FreeRTOS 业务封装同时运行。
8. 当前 `main_freertos.c` 中 `NRF24_STAGE1_TEST_ENABLED=1` 时不会启动
   FreeRTOS；接入业务任务前需要关闭该阶段测试。
