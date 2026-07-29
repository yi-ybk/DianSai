/**
 * @file    hc12_uart.c
 * @brief   HC-12 transparent UART implementation for MSPM0G3507
 * @brief   MSPM0G3507 的 HC-12 透明串口接口实现
 */
#include "hc12_uart.h"

#include "hc12_config.h"
#include "ti_msp_dl_config.h"

#include <ti/driverlib/driverlib.h>

/* Compile-time checks keep ring-buffer indexes within uint16_t.
 * 编译期检查保证环形缓冲区下标可由 uint16_t 表示。 */
_Static_assert(HC12_RX_BUFFER_SIZE > 1U,
               "HC12_RX_BUFFER_SIZE must be greater than one");
_Static_assert(HC12_RX_BUFFER_SIZE <= UINT16_MAX,
               "HC12_RX_BUFFER_SIZE must fit in uint16_t");

static volatile bool s_initialized;
static uint8_t s_rx_buffer[HC12_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint16_t s_rx_count;
static volatile uint32_t s_rx_overflow_count;
static Hc12RxNotifyFromIsr s_rx_notify_from_isr;
static void *s_rx_notify_context;

static uint32_t Hc12EnterCritical(void);
static void Hc12ExitCritical(uint32_t primask);
static void Hc12ResetBuffer(void);
static void Hc12PushRxByte(uint8_t value);

void Hc12Init(void)
{
    uint32_t primask = Hc12EnterCritical();

    Hc12ResetBuffer();
    while (!DL_UART_isRXFIFOEmpty(UART_HC12_INST))
        (void)DL_UART_receiveData(UART_HC12_INST);

    s_initialized = true;
    Hc12ExitCritical(primask);

    DL_UART_clearInterruptStatus(UART_HC12_INST, DL_UART_INTERRUPT_RX);
    DL_UART_enableInterrupt(UART_HC12_INST, DL_UART_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_HC12_INST_INT_IRQN);
    NVIC_SetPriority(UART_HC12_INST_INT_IRQN, HC12_UART_IRQ_PRIORITY);
    NVIC_EnableIRQ(UART_HC12_INST_INT_IRQN);
}

void Hc12Deinit(void)
{
    uint32_t primask;

    NVIC_DisableIRQ(UART_HC12_INST_INT_IRQN);
    DL_UART_disableInterrupt(UART_HC12_INST, DL_UART_INTERRUPT_RX);

    primask = Hc12EnterCritical();
    s_initialized = false;
    Hc12ResetBuffer();
    Hc12ExitCritical(primask);
}

bool Hc12IsInitialized(void)
{
    return s_initialized;
}

bool Hc12Write(const uint8_t *data, size_t length)
{
    size_t i;

    if (!s_initialized || (data == NULL) || (length == 0U))
        return false;

    for (i = 0U; i < length; i++)
        DL_UART_transmitDataBlocking(UART_HC12_INST, data[i]);

    return true;
}

bool Hc12WriteString(const char *text)
{
    if (!s_initialized || (text == NULL) || (*text == '\0'))
        return false;

    while (*text != '\0')
    {
        DL_UART_transmitDataBlocking(UART_HC12_INST, (uint8_t)*text);
        text++;
    }

    return true;
}

size_t Hc12Available(void)
{
    uint32_t primask;
    size_t available;

    primask = Hc12EnterCritical();
    available = s_rx_count;
    Hc12ExitCritical(primask);
    return available;
}

size_t Hc12Read(uint8_t *data, size_t capacity)
{
    uint32_t primask;
    size_t copied = 0U;

    if (!s_initialized || (data == NULL) || (capacity == 0U))
        return 0U;

    primask = Hc12EnterCritical();
    while ((copied < capacity) && (s_rx_count > 0U))
    {
        data[copied] = s_rx_buffer[s_rx_tail];
        copied++;
        s_rx_tail++;
        if (s_rx_tail >= HC12_RX_BUFFER_SIZE)
            s_rx_tail = 0U;
        s_rx_count--;
    }
    Hc12ExitCritical(primask);

    return copied;
}

bool Hc12ReadByte(uint8_t *value)
{
    return Hc12Read(value, 1U) == 1U;
}

void Hc12FlushRx(void)
{
    uint32_t primask = Hc12EnterCritical();

    Hc12ResetBuffer();
    while (!DL_UART_isRXFIFOEmpty(UART_HC12_INST))
        (void)DL_UART_receiveData(UART_HC12_INST);

    Hc12ExitCritical(primask);
}

uint32_t Hc12GetAndClearOverflowCount(void)
{
    uint32_t primask;
    uint32_t count;

    primask = Hc12EnterCritical();
    count = s_rx_overflow_count;
    s_rx_overflow_count = 0U;
    Hc12ExitCritical(primask);
    return count;
}

uint32_t Hc12GetOverflowCount(void)
{
    uint32_t primask;
    uint32_t count;

    primask = Hc12EnterCritical();
    count = s_rx_overflow_count;
    Hc12ExitCritical(primask);
    return count;
}

void Hc12SetRxNotifyFromIsr(
    Hc12RxNotifyFromIsr callback, void *context)
{
    uint32_t primask = Hc12EnterCritical();

    s_rx_notify_from_isr = callback;
    s_rx_notify_context = context;
    Hc12ExitCritical(primask);
}

void UART_HC12_INST_IRQHandler(void)
{
    Hc12RxNotifyFromIsr notify;
    void *notify_context;
    DL_UART_IIDX interrupt;
    bool received = false;

    interrupt = DL_UART_getPendingInterrupt(UART_HC12_INST);
    if (interrupt != DL_UART_IIDX_RX)
        return;

    while (!DL_UART_isRXFIFOEmpty(UART_HC12_INST))
    {
        Hc12PushRxByte(DL_UART_receiveData(UART_HC12_INST));
        received = true;
    }

    notify = s_rx_notify_from_isr;
    notify_context = s_rx_notify_context;
    if (received && (notify != NULL))
        notify(notify_context);
}

static uint32_t Hc12EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void Hc12ExitCritical(uint32_t primask)
{
    if (primask == 0U)
        __enable_irq();
}

static void Hc12ResetBuffer(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;
    s_rx_overflow_count = 0U;
}

static void Hc12PushRxByte(uint8_t value)
{
    if (!s_initialized)
        return;

    if (s_rx_count >= HC12_RX_BUFFER_SIZE)
    {
        /* Preserve newest data by discarding the oldest buffered byte.
         * 缓冲区满时丢弃最旧字节，以保留最新接收数据。 */
        s_rx_tail++;
        if (s_rx_tail >= HC12_RX_BUFFER_SIZE)
            s_rx_tail = 0U;
        s_rx_count--;
        s_rx_overflow_count++;
    }

    s_rx_buffer[s_rx_head] = value;
    s_rx_head++;
    if (s_rx_head >= HC12_RX_BUFFER_SIZE)
        s_rx_head = 0U;
    s_rx_count++;
}
