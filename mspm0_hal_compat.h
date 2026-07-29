#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <ti/driverlib/driverlib.h>
#include <FreeRTOS.h>
#include <task.h>

typedef GPIO_Regs GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0U, GPIO_PIN_SET = 1U } GPIO_PinState;
typedef enum { HAL_OK = 0U, HAL_ERROR = 1U } HAL_StatusTypeDef;

#define GPIO_PIN_0  DL_GPIO_PIN_0
#define GPIO_PIN_1  DL_GPIO_PIN_1
#define GPIO_PIN_2  DL_GPIO_PIN_2
#define GPIO_PIN_3  DL_GPIO_PIN_3
#define GPIO_PIN_4  DL_GPIO_PIN_4
#define GPIO_PIN_5  DL_GPIO_PIN_5
#define GPIO_PIN_6  DL_GPIO_PIN_6
#define GPIO_PIN_7  DL_GPIO_PIN_7
#define GPIO_PIN_8  DL_GPIO_PIN_8
#define GPIO_PIN_9  DL_GPIO_PIN_9
#define GPIO_PIN_10 DL_GPIO_PIN_10
#define GPIO_PIN_11 DL_GPIO_PIN_11
#define GPIO_PIN_12 DL_GPIO_PIN_12
#define GPIO_PIN_13 DL_GPIO_PIN_13
#define GPIO_PIN_14 DL_GPIO_PIN_14
#define GPIO_PIN_15 DL_GPIO_PIN_15
#define GPIO_PIN_16 DL_GPIO_PIN_16
#define GPIO_PIN_17 DL_GPIO_PIN_17
#define GPIO_PIN_18 DL_GPIO_PIN_18
#define GPIO_PIN_19 DL_GPIO_PIN_19
#define GPIO_PIN_20 DL_GPIO_PIN_20
#define GPIO_PIN_21 DL_GPIO_PIN_21
#define GPIO_PIN_22 DL_GPIO_PIN_22
#define GPIO_PIN_23 DL_GPIO_PIN_23
#define GPIO_PIN_24 DL_GPIO_PIN_24
#define GPIO_PIN_25 DL_GPIO_PIN_25
#define GPIO_PIN_26 DL_GPIO_PIN_26
#define GPIO_PIN_27 DL_GPIO_PIN_27
#define GPIO_PIN_28 DL_GPIO_PIN_28
#define GPIO_PIN_29 DL_GPIO_PIN_29
#define GPIO_PIN_30 DL_GPIO_PIN_30
#define GPIO_PIN_31 DL_GPIO_PIN_31

typedef struct { uint32_t Pin; uint32_t Mode; uint32_t Pull; uint32_t Speed; } GPIO_InitTypeDef;
#define GPIO_MODE_OUTPUT_OD 0U
#define GPIO_PULLUP 0U
#define GPIO_SPEED_FREQ_HIGH 0U

typedef struct {
    GPTIMER_Regs *Instance;
    struct { uint32_t Prescaler; uint32_t Period; } Init;
    uint32_t Channel;
    uint32_t clock_hz;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_CHANNEL_ALL 0xFFFFFFFFU

typedef struct { struct { uint32_t Mode; } Init; } DMA_HandleTypeDef;
#define DMA_CIRCULAR 1U
#define DMA_IT_HT 0U
typedef struct { UART_Regs *Instance; DMA_HandleTypeDef *hdmarx; uint32_t gState; } UART_HandleTypeDef;
typedef uint32_t HAL_UART_StateTypeDef;
#define HAL_UART_STATE_BUSY_TX 1U
#define HAL_UART_STATE_BUSY_TX_RX 2U
#define HAL_UART_RXEVENT_TC 0U

typedef struct { I2C_Regs *Instance; uint16_t Devaddress; } I2C_HandleTypeDef;
#define HAL_I2C_MODULE_ENABLED
#define I2C_OTHER_AND_LAST_FRAME 0U
#define I2C_OTHER_FRAME 0U
#define I2C_MEMADD_SIZE_8BIT 1U
#define I2C_MEMADD_SIZE_16BIT 2U

static inline void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint32_t pins, GPIO_PinState state) { if (state == GPIO_PIN_SET) { DL_GPIO_setPins(gpio, pins); } else { DL_GPIO_clearPins(gpio, pins); } }
static inline void HAL_GPIO_TogglePin(GPIO_TypeDef *gpio, uint32_t pins) { DL_GPIO_togglePins(gpio, pins); }
static inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio, uint32_t pins) { return (DL_GPIO_readPins(gpio, pins) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
static inline void HAL_GPIO_Init(GPIO_TypeDef *gpio, GPIO_InitTypeDef *config) { (void)gpio; (void)config; }
static inline uint32_t HAL_GetTick(void) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }
static inline uint32_t HAL_GetTickFromISR(void) { return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS); }

static inline HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *timer) { if ((timer == NULL) || (timer->Instance == NULL)) return HAL_ERROR; DL_TimerG_startCounter(timer->Instance); return HAL_OK; }
static inline HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *timer) { if ((timer == NULL) || (timer->Instance == NULL)) return HAL_ERROR; DL_TimerG_stopCounter(timer->Instance); return HAL_OK; }
#define HAL_TIM_Base_Start_IT HAL_TIM_Base_Start
#define HAL_TIM_Base_Stop_IT HAL_TIM_Base_Stop
#define HAL_TIM_Encoder_Start(timer, channel) HAL_TIM_Base_Start(timer)
#define HAL_TIM_Encoder_Stop(timer, channel) HAL_TIM_Base_Stop(timer)
#define HAL_TIM_Encoder_Start_IT(timer, channel) HAL_TIM_Base_Start(timer)
#define HAL_TIM_Encoder_Stop_IT(timer, channel) HAL_TIM_Base_Stop(timer)
#define HAL_TIM_PWM_Start(timer, channel) HAL_TIM_Base_Start(timer)
#define HAL_TIM_PWM_Stop(timer, channel) HAL_TIM_Base_Stop(timer)
#define HAL_TIM_PWM_Start_DMA(timer, channel, data, size) HAL_ERROR
#define __HAL_TIM_GET_COUNTER(timer) DL_TimerG_getTimerCount((timer)->Instance)
#define __HAL_TIM_SET_COUNTER(timer, value) DL_TimerG_setTimerCount((timer)->Instance, (value))
#define __HAL_TIM_SET_AUTORELOAD(timer, value) do { DL_TimerG_setLoadValue((timer)->Instance, (value)); (timer)->Init.Period = (value); } while (0)
#define __HAL_TIM_SetAutoreload __HAL_TIM_SET_AUTORELOAD
#define __HAL_TIM_SetCompare(timer, channel, value) DL_TimerG_setCaptureCompareValue((timer)->Instance, (value), (DL_TIMER_CC_INDEX)((channel) / 4U))
#define __HAL_TIM_IS_TIM_COUNTING_DOWN(timer) 0U

static inline HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size, uint32_t timeout) { uint16_t i; (void)timeout; if ((uart == NULL) || (uart->Instance == NULL)) return HAL_ERROR; for (i = 0U; i < size; ++i) DL_UART_transmitDataBlocking(uart->Instance, data[i]); return HAL_OK; }
#define HAL_UART_Transmit_IT(uart, data, size) HAL_ERROR
#define HAL_UART_Transmit_DMA(uart, data, size) HAL_ERROR
#define HAL_UARTEx_ReceiveToIdle_DMA(uart, data, size) HAL_ERROR
#define HAL_UARTEx_GetRxEventType(uart) HAL_UART_RXEVENT_TC
#define __HAL_DMA_DISABLE_IT(handle, flag) do { (void)(handle); (void)(flag); } while (0)

static inline HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *i2c, uint16_t address, uint8_t *data, uint16_t size, uint32_t timeout) { (void)i2c; (void)address; (void)data; (void)size; (void)timeout; return HAL_ERROR; }
static inline HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *i2c, uint16_t address, uint8_t *data, uint16_t size, uint32_t timeout) { (void)i2c; (void)address; (void)data; (void)size; (void)timeout; return HAL_ERROR; }
#define HAL_I2C_Master_Seq_Transmit_IT(i2c, address, data, size, frame) HAL_ERROR
#define HAL_I2C_Master_Seq_Transmit_DMA(i2c, address, data, size, frame) HAL_ERROR
#define HAL_I2C_Master_Seq_Receive_IT(i2c, address, data, size, frame) HAL_ERROR
#define HAL_I2C_Master_Seq_Receive_DMA(i2c, address, data, size, frame) HAL_ERROR
#define HAL_I2C_Mem_Write(i2c, address, mem, memsize, data, size, timeout) HAL_ERROR
#define HAL_I2C_Mem_Read(i2c, address, mem, memsize, data, size, timeout) HAL_ERROR

static inline uint32_t HAL_RCC_GetPCLK1Freq(void) { return 0U; }
static inline uint32_t HAL_RCC_GetPCLK2Freq(void) { return 0U; }
#define __HAL_RCC_GPIOA_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOG_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOI_CLK_ENABLE() do {} while (0)
