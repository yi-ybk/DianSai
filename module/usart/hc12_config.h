/**
 * @file    hc12_config.h
 * @brief   HC-12 UART configuration for the Tianmengxing MSPM0G3507 board
 * @brief   天猛星 MSPM0G3507 开发板的 HC-12 串口配置
 */
#pragma once

#include <stdint.h>

/**
 * @brief UART baud rate configured in SysConfig.
 * @brief SysConfig 中配置的串口波特率。
 *
 * This value documents the required HC-12 transparent-mode baud rate.
 * The actual MSPM0 UART divider is generated from DianSai_MSPM0G3507.syscfg.
 *
 * 该值记录 HC-12 透明传输模式所需的波特率。MSPM0 实际串口分频参数由
 * DianSai_MSPM0G3507.syscfg 生成。
 */
#define HC12_UART_BAUD_RATE 9600U

/** Receive ring-buffer capacity. / 接收环形缓冲区容量。 */
#define HC12_RX_BUFFER_SIZE 256U

/** Cortex-M interrupt priority used by UART3. / UART3 使用的 Cortex-M 中断优先级。 */
#define HC12_UART_IRQ_PRIORITY 2U
