# 通用从机串口通信组件使用说明

## 1. 组件职责

`Slaver_t` 位于应用层，用于实现从机侧串口通信。组件负责DMA接收、字节流缓存、帧头同步、
分帧、帧校验和原始帧发送；具体通信协议和业务数据由应用层回调定义。

接收数据流如下：

```text
UART DMA/IDLE中断 -> 静态流缓冲 -> SlaverProcess或SlaverTask -> 协议回调
```

DMA回调只搬运字节，不解析协议，也不执行用户业务回调。帧长度、校验和业务处理均在任务
上下文执行。

本组件没有固定帧头、帧尾、命令字、校验方式或数据类型。它可以处理整数、浮点数、二进制
结构、字符串或自定义TLV数据。

## 2. 使用前提

- 在SysConfig中为从机通信配置一个UART、其RX DMA、DMA完成中断和RX超时中断。
- UART应使用与主机一致的波特率、数据位、校验和停止位。
- 一个UART只能被一个 `USARTRegister()` 用户占用。
- 当前工程已经配置 `UART_SLAVER`：UART3、PB12/TX、PB13/RX、115200 baud、DMA通道1。
- `Slaver_t` 依赖FreeRTOS流缓冲，必须在FreeRTOS可用的工程中使用。

`SlaverInit()` 会检查UART是否已被USART模块注册。若已占用，初始化返回 `false`，不会
强行抢占该UART。

## 3. 核心接口

| 接口 | 作用 |
|---|---|
| `init()` | 注册UART DMA接收并初始化协议解析状态 |
| `process()` | 处理当前已经接收的字节，必须在任务上下文调用 |
| `task()` | 内置解析任务入口，参数传入 `Slaver_t *` |
| `send()` | 阻塞发送上层组装好的完整原始帧 |
| `get_data()` | 获取收发统计数据的一致快照 |

`process()` 和 `task()` 只能二选一：使用内置任务时不要在另一任务中再次调用 `process()`。

## 4. 协议回调

初始化时必须提供两个回调：

- `frame_length_callback`：根据当前候选帧和已接收字节数确定完整帧长度。
- `frame_callback`：处理一帧校验通过的完整数据。

`frame_validate_callback` 可选，用于检查帧尾、命令字、和校验或CRC。返回 `false` 时该帧被
丢弃，`validate_error_count` 增加。

帧长度回调返回值含义：

| 返回值 | 含义 |
|---|---|
| `SLAVER_FRAME_WAIT` | 数据尚未收完整，保留当前缓存等待后续字节 |
| `SLAVER_FRAME_READY` | `frame_length` 已给出且完整帧已到达 |
| `SLAVER_FRAME_INVALID` | 当前帧起始数据不合法，组件丢弃一个字节后重新同步 |

传入业务回调的 `frame` 指针指向内部解析缓存，仅在本次回调期间有效。需要异步保存时，
必须由应用层复制实际需要的数据。

## 5. 长度字段辅助函数

`SlaverFrameLengthFromLengthField()` 支持常见的1字节或2字节长度字段。其配置项：

| 参数 | 含义 |
|---|---|
| `length_offset` | 长度字段相对帧起始的偏移 |
| `length_size` | 长度字段大小，仅支持1或2字节 |
| `byte_order` | 2字节长度字段的大小端字节序 |
| `length_is_payload` | `true` 表示字段值仅表示载荷长度 |
| `frame_overhead` | 载荷长度模式下，帧头、命令、长度、校验和帧尾的总长度 |
| `max_frame_length` | 该协议允许的最大完整帧长度，不能超过 `SLAVER_PARSE_BUFFER_SIZE` |

若协议不是长度字段形式，例如固定长度帧、结束符帧或TLV嵌套帧，直接自行实现
`SlaverFrameLengthCallback_t` 即可。

## 6. 参考协议示例

以下示例仅用于说明配置，不是模块规定的通信协议：

```text
0x55 0xAA | command(1) | payload_length(1) | payload(N) | checksum(1) | 0x0D 0x0A
```

完整帧长度为：

```text
2 + 1 + 1 + N + 1 + 2 = N + 7
```

定义协议上下文和回调：

```c
#include "slaver.h"
#include "user_lib.h"

typedef struct
{
    SlaverLengthFieldConfig_t length_config;
} UserProtocol_t;

static UserProtocol_t user_protocol = {
    .length_config = {
        .length_offset = 3U,
        .length_size = 1U,
        .byte_order = SLAVER_BYTE_ORDER_LITTLE_ENDIAN,
        .length_is_payload = true,
        .frame_overhead = 7U,
        .max_frame_length = 128U,
    },
};

static SlaverFrameState_t UserFrameLength(const uint8_t *frame,
                                          uint16_t available,
                                          uint16_t *frame_length,
                                          void *context)
{
    UserProtocol_t *protocol = (UserProtocol_t *)context;

    return SlaverFrameLengthFromLengthField(frame,
                                             available,
                                             frame_length,
                                             &protocol->length_config);
}

static bool UserFrameValidate(const uint8_t *frame,
                              uint16_t frame_length,
                              void *context)
{
    (void)context;

    /* 此处替换为实际的CRC、和校验或命令字校验。 */
    return (frame_length >= 7U) &&
           (frame[frame_length - 2U] == 0x0DU) &&
           (frame[frame_length - 1U] == 0x0AU);
}

static void UserFrameReceived(Slaver_t *slaver,
                              const uint8_t *frame,
                              uint16_t frame_length,
                              void *context)
{
    uint8_t command = frame[2];
    uint8_t payload_length = frame[3];
    const uint8_t *payload = &frame[4];

    (void)slaver;
    (void)frame_length;
    (void)context;

    if ((command == 0x01U) && (payload_length >= 4U))
    {
        float value = BytesToFloatLE(payload);

        /* 使用value更新应用层数据。 */
        (void)value;
    }
}
```

`BytesToFloatLE()`、`BytesToFloatBE()`、`BytesToUint16LE()`、`BytesToUint32BE()` 等
字节转换函数由 `module/algorithm/user_lib.h` 提供。只有协议字段确实使用IEEE 754
浮点数时才应调用浮点转换函数。

## 7. 固定长度float协议

模块已提供固定7字节float协议辅助接口，帧格式如下：

```text
帧头(1) | float小端序(4) | 校验和(1) | 帧尾(1)
```

校验和为帧头和4个float数据字节的8位累加和，不包含校验和自身和帧尾。该协议没有应用层帧队列；
每接收一帧校验通过的数据，只会覆盖保存 `protocol.latest.value`，适合只关心最新控制量的场景。

```c
static SlaverSimpleFloatProtocol_t protocol;

static const SlaverSimpleFloatProtocolConfig_t protocol_config = {
    .frame_header = 0xA5U,
    .frame_tail = 0x5AU,
    .frame_length = 7U,
};

(void)SlaverSimpleFloatProtocolInit(&protocol, &protocol_config);

static const SlaverInitConfig_t slaver_config = {
    .header = {0xA5U},
    .header_length = 1U,
    .frame_length_callback = SlaverSimpleFloatFrameLength,
    .frame_validate_callback = SlaverSimpleFloatFrameValidate,
    .frame_callback = SlaverSimpleFloatFrameReceived,
    .protocol_context = &protocol,
    /* UART、DMA和中断配置见下一节。 */
};
```

在其他任务中读取最新值：

```c
SlaverSimpleFloatData_t latest;

SlaverSimpleFloatGetLatest(&protocol, &latest);
if (latest.valid)
{
    float control_value = latest.value;
    /* 使用control_value。 */
}
```

发送一帧：

```c
(void)SlaverSimpleFloatSend(&slaver, &protocol, 12.5f);
```

帧头、帧尾和帧长由 `SlaverSimpleFloatProtocolConfig_t` 配置；当前float协议的帧长必须为7。
`SlaverInitConfig_t.header` 用于底层帧同步，应与 `protocol_config.frame_header` 保持一致。

## 8. 初始化和任务启动

先定义对象与初始化配置：

```c
Slaver_t slaver = {SLAVER_OBJECT_DEFAULT};

static UART_HandleTypeDef huart_slaver = {
    .Instance = UART_SLAVER_INST,
};

static const SlaverInitConfig_t slaver_config = {
    .uart_handle = &huart_slaver,
    .dma_config = {
        .dma = DMA,
        .rx_channel = DMA_CH_SLAVER_RX_CHAN_ID,
        .tx_channel = USART_DMA_CHANNEL_INVALID,
    },
    .uart_irqn = UART_SLAVER_INST_INT_IRQN,
    .uart_irq_priority = 2U,
    .dma_rx_buffer_size = 128U,
    .header = {0x55U, 0xAAU},
    .header_length = 2U,
    .frame_length_callback = UserFrameLength,
    .frame_validate_callback = UserFrameValidate,
    .frame_callback = UserFrameReceived,
    .protocol_context = &user_protocol,
    .tx_timeout_ms = 100U,
};
```

由于接收中断会调用FreeRTOS的 `FromISR` 接口，应在调度器启动后的任务上下文完成初始化。
可以创建一个包装任务，在其中初始化后直接进入模块任务：

```c
static void SlaverStartTask(void *argument)
{
    Slaver_t *instance = (Slaver_t *)argument;

    if ((instance == NULL) || !instance->init(instance, &slaver_config))
    {
        vTaskDelete(NULL);
        return;
    }

    instance->task(instance);
}
```

在应用层实现SysConfig生成的UART中断入口：

```c
void UART_SLAVER_INST_IRQHandler(void)
{
    USARTIRQHandler(&huart_slaver);
}
```

推荐使用内置任务：

```c
static const osThreadAttr_t slaver_task_attributes = {
    .name = "slaverTask",
    .stack_size = 512U * 4U,
    .priority = osPriorityNormal,
};

osThreadId_t slaver_task_handle =
    osThreadNew(SlaverStartTask, &slaver, &slaver_task_attributes);
```

也可以由已有任务调用：

```c
for (;;)
{
    slaver.process(&slaver);
    osDelay(5U);
}
```

不要同时创建 `SlaverTask` 和调用 `slaver.process()` 的外部任务。

## 9. 发送数据

`send()` 不自动添加帧头、长度、校验或帧尾。应用层应先按自己的协议组帧，再发送：

```c
uint8_t frame[] = {0x55U, 0xAAU, 0x10U, 0x00U, 0x00U, 0x0DU, 0x0AU};

if (!slaver.send(&slaver, frame, sizeof(frame)))
{
    /* 发送超时或UART状态异常处理 */
}
```

`send()` 为阻塞调用，不应在中断回调中使用。多个任务同时发送时，后发任务可能因UART忙而
失败；需要多任务连续发送时，应由应用层建立单一发送任务或发送队列。

## 10. 获取状态与排查

```c
SlaverData_t slaver_data;

slaver.get_data(&slaver, &slaver_data);
```

| 字段 | 含义 |
|---|---|
| `rx_byte_count` | DMA回调接收到的总字节数 |
| `rx_frame_count` | 成功分发给业务回调的完整帧数 |
| `invalid_frame_count` | 帧头同步失败或长度不合法次数 |
| `validate_error_count` | 通过长度检查但校验回调失败次数 |
| `rx_stream_drop_count` | 任务处理不及时导致的流缓冲丢弃次数 |
| `tx_frame_count` / `tx_error_count` | 发送成功/失败次数 |

常见问题：

- `rx_byte_count` 为0：检查UART、DMA、NVIC中断、串口接线和波特率。
- `rx_byte_count` 增加但 `rx_frame_count` 为0：检查帧头、长度字段偏移、长度单位和校验回调。
- `validate_error_count` 持续增加：检查帧尾、CRC算法、CRC覆盖范围和字节序。
- `rx_stream_drop_count` 增加：业务回调或解析任务运行过慢，应缩短回调执行时间或增大缓冲。
- `SlaverInit()` 返回 `false`：检查UART是否已经被IMU等模块注册，或配置范围是否超过
  `USART_RXBUFF_LIMIT`、`SLAVER_MAX_HEADER_LENGTH`。

## 11. 使用约束

- `header_length` 范围为1至 `SLAVER_MAX_HEADER_LENGTH`。
- 单帧最大长度不能超过 `SLAVER_PARSE_BUFFER_SIZE`，当前为256字节。
- DMA接收缓存大小不能超过 `USART_RXBUFF_LIMIT`，当前为256字节。
- UART中断优先级必须满足FreeRTOS调用 `xStreamBufferSendFromISR()` 的要求；本工程建议使用2。
- 业务帧回调不在中断中运行，但仍应避免长时间阻塞，否则可能导致流缓冲溢出。
- `Slaver_t` 包含接收和解析缓存，建议定义为静态或全局对象，不要定义为短生命周期的局部变量。
