/**
 * @file    slaver.h
 * @brief   通用从机串口通信组件头文件
 * @details 基于DMA接收和FreeRTOS流缓冲实现可配置的变长帧接收与发送。
 */
#pragma once

#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include <stdbool.h>
#include <stdint.h>

#define SLAVER_MAX_HEADER_LENGTH 8U
#define SLAVER_RX_STREAM_SIZE 512U
#define SLAVER_PARSE_BUFFER_SIZE 256U
#define SLAVER_TASK_DELAY_MS 5U

/** @brief 帧长度解析结果 */
typedef enum
{
    SLAVER_FRAME_WAIT = 0, /**< 当前字节不足以确定完整帧 */
    SLAVER_FRAME_READY,    /**< 已得到完整帧长度 */
    SLAVER_FRAME_INVALID,  /**< 当前帧起始数据不合法 */
} SlaverFrameState_t;

/** @brief 长度字段字节序 */
typedef enum
{
    SLAVER_BYTE_ORDER_LITTLE_ENDIAN = 0, /**< 小端字节序 */
    SLAVER_BYTE_ORDER_BIG_ENDIAN,        /**< 大端字节序 */
} SlaverByteOrder_t;

/** @brief 长度字段通用解析配置 */
typedef struct
{
    uint16_t length_offset;       /**< 长度字段相对帧起始的偏移 */
    uint8_t length_size;          /**< 长度字段字节数，仅支持1或2 */
    SlaverByteOrder_t byte_order; /**< 长度字段字节序 */
    bool length_is_payload;       /**< true表示字段值为载荷长度 */
    uint16_t frame_overhead;      /**< 载荷长度模式下的帧头、校验和帧尾总长度 */
    uint16_t max_frame_length;    /**< 协议允许的最大完整帧长度 */
} SlaverLengthFieldConfig_t;

typedef struct Slaver Slaver_t;

/**
 * @brief   解析当前候选帧长度
 * @param   frame 当前帧起始地址
 * @param   available 当前可用字节数
 * @param   frame_length 输出完整帧长度
 * @param   context 用户协议上下文
 * @return  SlaverFrameState_t 解析结果
 * @note    回调运行于任务上下文，不在串口中断中执行。
 */
typedef SlaverFrameState_t (*SlaverFrameLengthCallback_t)(const uint8_t *frame,
                                                           uint16_t available,
                                                           uint16_t *frame_length,
                                                           void *context);

/**
 * @brief   校验完整帧合法性
 * @return  bool 校验通过返回true
 * @note    可用于实现帧尾、和校验、CRC或命令字检查；为NULL时跳过校验。
 */
typedef bool (*SlaverFrameValidateCallback_t)(const uint8_t *frame,
                                              uint16_t frame_length,
                                              void *context);

/**
 * @brief   处理一帧完整数据
 * @note    回调运行于调用process()或task()的任务上下文，可进行业务处理但不应长时间阻塞。
 */
typedef void (*SlaverFrameCallback_t)(Slaver_t *slaver,
                                      const uint8_t *frame,
                                      uint16_t frame_length,
                                      void *context);

/** @brief 从机通信初始化配置 */
typedef struct
{
    UART_HandleTypeDef *uart_handle;                        /**< 绑定的UART句柄，必须未被其他USART模块注册 */
    USART_DMA_Config_s dma_config;                          /**< MSPM0 UART接收DMA配置 */
    IRQn_Type uart_irqn;                                    /**< SysConfig生成的UART中断号 */
    uint32_t uart_irq_priority;                             /**< UART中断优先级，必须允许调用FreeRTOS FromISR接口 */
    uint16_t dma_rx_buffer_size;                            /**< DMA循环接收缓冲区大小，范围1~USART_RXBUFF_LIMIT */
    uint8_t header[SLAVER_MAX_HEADER_LENGTH];               /**< 帧头内容 */
    uint8_t header_length;                                  /**< 帧头长度，范围1~SLAVER_MAX_HEADER_LENGTH */
    SlaverFrameLengthCallback_t frame_length_callback;      /**< 帧长度解析回调 */
    SlaverFrameValidateCallback_t frame_validate_callback;  /**< 完整帧校验回调，可为NULL */
    SlaverFrameCallback_t frame_callback;                   /**< 完整帧业务回调 */
    void *protocol_context;                                 /**< 传给协议回调的用户上下文 */
    uint32_t tx_timeout_ms;                                 /**< 阻塞发送超时时间(ms)，0时使用100 ms */
} SlaverInitConfig_t;

/** @brief 从机通信运行数据 */
typedef struct
{
    volatile uint32_t rx_byte_count;           /**< DMA接收的总字节数 */
    volatile uint32_t rx_stream_drop_count;    /**< 流缓冲写入失败次数 */
    volatile uint32_t rx_stream_drop_bytes;    /**< 流缓冲丢弃的字节数 */
    uint32_t rx_frame_count;                   /**< 成功解析的帧数 */
    uint32_t invalid_frame_count;              /**< 帧长或帧头错误次数 */
    uint32_t validate_error_count;             /**< 完整帧校验失败次数 */
    uint32_t tx_frame_count;                   /**< 成功发送的帧数 */
    uint32_t tx_error_count;                   /**< 发送失败次数 */
    uint32_t last_rx_tick;                     /**< 最近接收字节的时间戳 */
    uint32_t last_frame_tick;                  /**< 最近成功解析帧的时间戳 */
} SlaverData_t;

/** @brief 简单浮点协议配置 */
typedef struct
{
    uint8_t frame_header;  /**< 帧头 */
    uint8_t frame_tail;    /**< 帧尾 */
    uint16_t frame_length; /**< 帧总长度；当前 float 协议固定为 7 字节 */
} SlaverSimpleFloatProtocolConfig_t;

/** @brief 简单浮点协议保存的最新数据 */
typedef struct
{
    float value;                 /**< 最近一次校验通过的float数据 */
    uint32_t update_count;       /**< 最近数据被更新的次数 */
    uint32_t last_update_tick;   /**< 最近数据更新时间(ms) */
    bool valid;                  /**< 是否已接收到至少一帧有效数据 */
} SlaverSimpleFloatData_t;

/**
 * @brief 简单浮点协议上下文
 * @details 固定帧格式为：帧头(1) + float小端序(4) + 累加校验和(1) + 帧尾(1)。
 */
typedef struct
{
    SlaverSimpleFloatProtocolConfig_t config; /**< 协议帧配置 */
    SlaverSimpleFloatData_t latest;     /**< 仅保存最新的有效帧数据 */
} SlaverSimpleFloatProtocol_t;

/** @brief 从机串口通信对象 */
struct Slaver
{
    USARTInstance *usart;                               /**< 底层USART对象 */
    bool initialized;                                   /**< 初始化标记 */
    SlaverInitConfig_t init_config;                     /**< 初始化配置缓存 */
    SlaverData_t data;                                  /**< 当前运行数据 */
    uint8_t parse_buffer[SLAVER_PARSE_BUFFER_SIZE];     /**< 待解析字节缓存 */
    uint16_t parse_length;                              /**< 待解析字节数 */
    StreamBufferHandle_t rx_stream;                     /**< 内部静态流缓冲句柄 */
    uint8_t rx_stream_storage[SLAVER_RX_STREAM_SIZE];   /**< 内部静态流缓冲存储区 */
    StaticStreamBuffer_t rx_stream_control_block;       /**< 内部静态流缓冲控制块 */

    bool (*init)(Slaver_t *slaver, const SlaverInitConfig_t *config);
    void (*process)(Slaver_t *slaver);
    void (*task)(void *argument);
    bool (*send)(Slaver_t *slaver, const uint8_t *data, uint16_t length);
    void (*get_data)(Slaver_t *slaver, SlaverData_t *data);
};

/** @brief 初始化从机串口通信对象 */
bool SlaverInit(Slaver_t *slaver, const SlaverInitConfig_t *config);
/** @brief 处理已经接收到的串口数据，应在任务上下文调用 */
void SlaverProcess(Slaver_t *slaver);
/** @brief 从机通信任务入口，argument传入Slaver_t对象地址 */
void SlaverTask(void *argument);
/** @brief 阻塞发送完整原始帧 */
bool SlaverSend(Slaver_t *slaver, const uint8_t *data, uint16_t length);
/** @brief 获取从机通信运行数据的一致快照 */
void SlaverGetData(Slaver_t *slaver, SlaverData_t *data);

/** @brief 根据1或2字节长度字段解析完整帧长度 */
SlaverFrameState_t SlaverFrameLengthFromLengthField(const uint8_t *frame,
                                                     uint16_t available,
                                                     uint16_t *frame_length,
                                                     void *context);

/** @brief 初始化简单浮点协议上下文；当前协议帧长度必须配置为7字节 */
bool SlaverSimpleFloatProtocolInit(
    SlaverSimpleFloatProtocol_t *protocol,
    const SlaverSimpleFloatProtocolConfig_t *config);
/** @brief 简单浮点协议的固定帧长度回调 */
SlaverFrameState_t SlaverSimpleFloatFrameLength(const uint8_t *frame,
                                                uint16_t available,
                                                uint16_t *frame_length,
                                                void *context);
/** @brief 简单浮点协议的帧校验回调 */
bool SlaverSimpleFloatFrameValidate(const uint8_t *frame,
                                    uint16_t frame_length,
                                    void *context);
/** @brief 简单浮点协议的接收回调，仅覆盖保存最新数据 */
void SlaverSimpleFloatFrameReceived(Slaver_t *slaver,
                                    const uint8_t *frame,
                                    uint16_t frame_length,
                                    void *context);
/** @brief 按简单浮点协议发送一个float数据 */
bool SlaverSimpleFloatSend(Slaver_t *slaver,
                           const SlaverSimpleFloatProtocol_t *protocol,
                           float value);
/** @brief 获取简单浮点协议的最新数据快照 */
void SlaverSimpleFloatGetLatest(const SlaverSimpleFloatProtocol_t *protocol,
                                SlaverSimpleFloatData_t *data);

#define SLAVER_OBJECT_DEFAULT             \
    .init     = SlaverInit,               \
    .process  = SlaverProcess,            \
    .task     = SlaverTask,               \
    .send     = SlaverSend,               \
    .get_data = SlaverGetData
