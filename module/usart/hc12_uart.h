/**
 * @file    hc12_uart.h
 * @brief   HC-12 transparent UART API for MSPM0G3507
 * @brief   MSPM0G3507 的 HC-12 透明串口接口
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Callback invoked from the UART3 ISR after new bytes enter the buffer.
 * @brief UART3 中断将新数据写入缓冲区后调用的回调函数。
 *
 * The callback runs in interrupt context and may only call ISR-safe functions.
 * 回调在中断上下文运行，只能调用支持 FromISR 的接口。
 */
typedef void (*Hc12RxNotifyFromIsr)(void *context);

/**
 * @brief Initialize the HC-12 software state and enable the UART3 RX interrupt.
 * @brief 初始化 HC-12 软件状态并使能 UART3 接收中断。
 *
 * SYSCFG_DL_init() must be called before this function. UART3 is configured as
 * 9600-8-N-1 on PB12(TX)/PB13(RX) by DianSai_MSPM0G3507.syscfg.
 *
 * 调用本函数前必须先调用 SYSCFG_DL_init()。UART3 由
 * DianSai_MSPM0G3507.syscfg 配置为 9600-8-N-1，PB12 为 TX，PB13 为 RX。
 */
void Hc12Init(void);

/**
 * @brief Disable the UART3 RX interrupt and clear the software buffer.
 * @brief 关闭 UART3 接收中断并清空软件缓冲区。
 */
void Hc12Deinit(void);

/**
 * @brief Return whether Hc12Init has completed.
 * @brief 返回 Hc12Init 是否已经完成。
 */
bool Hc12IsInitialized(void);

/**
 * @brief Send binary data with the blocking UART transmitter.
 * @brief 使用阻塞串口发送二进制数据。
 * @param data Data to send. / 待发送数据。
 * @param length Data length in bytes. / 数据长度，单位字节。
 * @return true on success, false for invalid arguments or before initialization.
 *         / 成功返回 true；参数非法或尚未初始化时返回 false。
 */
bool Hc12Write(const uint8_t *data, size_t length);

/**
 * @brief Send one null-terminated string without its terminator.
 * @brief 发送一个以空字符结尾的字符串，不发送结尾空字符。
 * @param text String to send. / 待发送字符串。
 * @return true on success. / 成功返回 true。
 */
bool Hc12WriteString(const char *text);

/**
 * @brief Return the number of bytes waiting in the receive ring buffer.
 * @brief 返回接收环形缓冲区中的待读字节数。
 */
size_t Hc12Available(void);

/**
 * @brief Read up to capacity bytes without blocking.
 * @brief 非阻塞读取最多 capacity 个字节。
 * @param data Destination buffer. / 目标缓冲区。
 * @param capacity Destination-buffer capacity. / 目标缓冲区容量。
 * @return Number of bytes copied. / 实际复制的字节数。
 */
size_t Hc12Read(uint8_t *data, size_t capacity);

/**
 * @brief Read one byte without blocking.
 * @brief 非阻塞读取一个字节。
 * @param value Destination byte. / 目标字节。
 * @return true when one byte was read, false when no data is available.
 *         / 成功读到一个字节返回 true；无数据时返回 false。
 */
bool Hc12ReadByte(uint8_t *value);

/**
 * @brief Discard all bytes currently buffered by the software and UART FIFO.
 * @brief 丢弃软件缓冲区和 UART FIFO 中当前保存的所有字节。
 */
void Hc12FlushRx(void);

/**
 * @brief Return and clear the receive-overflow counter.
 * @brief 返回并清零接收溢出计数。
 *
 * When the software ring buffer is full, the oldest byte is discarded and the
 * counter is incremented.
 *
 * 软件环形缓冲区已满时会丢弃最旧字节，并增加该计数。
 */
uint32_t Hc12GetAndClearOverflowCount(void);

/**
 * @brief Return the receive-overflow counter without clearing it.
 * @brief 返回接收溢出计数，但不清零。
 */
uint32_t Hc12GetOverflowCount(void);

/**
 * @brief Register one optional receive notification callback.
 * @brief 注册一个可选的接收通知回调。
 *
 * @param callback ISR callback, or NULL to disable notification.
 *                 / ISR 回调；传入 NULL 可关闭通知。
 * @param context Opaque callback context. / 传递给回调的上下文。
 *
 * This does not transfer ownership of received bytes. Data remains in the
 * driver's ring buffer and must still be read with Hc12Read().
 * 本接口不会转移数据所有权；数据仍保存在驱动环形缓冲区中，需使用
 * Hc12Read() 读取。
 */
void Hc12SetRxNotifyFromIsr(Hc12RxNotifyFromIsr callback, void *context);
