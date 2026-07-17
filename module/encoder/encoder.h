/**
 * @file    encoder.h
 * @brief   编码器驱动模块头文件
 * @details 封装编码器初始化、启停、更新与数据读取接口，向上层提供统一对象式访问。
 */
#pragma once

#include "bsp_tim.h"
#include <stdbool.h>
#include <stdint.h>

#define ENCODER_SPEED_WINDOW_MAX_SAMPLES 16U

/** @brief 编码器计数方向 */
typedef enum
{
    ENCODER_DIR_STOP = 0,   /**< 停止 */
    ENCODER_DIR_FORWARD,    /**< 正向 */
    ENCODER_DIR_REVERSE,    /**< 反向 */
} EncoderDirection_t;

/** @brief 编码器初始化配置 */
typedef struct
{
    TIM_HandleTypeDef *htim; /**< 编码器对应定时器句柄 */
    uint32_t channel;        /**< 编码器通道，通常使用 TIM_CHANNEL_ALL */
    bool reversed;           /**< 是否反向计数 */
    float counts_per_rev;    /**< 每圈脉冲数 */
    bool auto_start;         /**< 初始化后是否自动启动 */
    uint8_t speed_window_samples; /**< Speed estimate window length; 1 uses one update. */
} EncoderInitConfig_t;

/** @brief 编码器运行数据 */
typedef struct
{
    bool enabled;                 /**< 是否使能 */
    EncoderDirection_t direction; /**< 当前方向 */
    int32_t count;                /**< 累计计数值 */
    int32_t delta;                /**< 本次更新增量 */
    uint32_t raw_count;           /**< 定时器原始计数值 */
    float speed_cps;              /**< 速度（count/s） */
    float speed_rps;              /**< 速度（rev/s） */
    float raw_speed_cps;          /**< Raw speed from the current update (count/s). */
} EncoderData_t;

typedef struct Encoder Encoder_t;

/** @brief 编码器对象 */
struct Encoder
{
    TIMInstance *tim;                /**< 底层定时器实例 */
    bool initialized;                /**< 初始化标记 */
    EncoderInitConfig_t init_config; /**< 初始化配置缓存 */
    EncoderData_t data;              /**< 当前运行数据 */

    int32_t speed_delta_history[ENCODER_SPEED_WINDOW_MAX_SAMPLES];
    float speed_dt_history[ENCODER_SPEED_WINDOW_MAX_SAMPLES];
    int32_t speed_delta_sum;
    float speed_dt_sum;
    uint8_t speed_window_index;
    uint8_t speed_window_count;

    bool (*init)(Encoder_t *encoder, const EncoderInitConfig_t *config);
    bool (*start)(Encoder_t *encoder);
    bool (*stop)(Encoder_t *encoder);
    void (*update)(Encoder_t *encoder, float dt_s);
    void (*reset)(Encoder_t *encoder);
    int32_t (*get_count)(Encoder_t *encoder);
    EncoderDirection_t (*get_direction)(Encoder_t *encoder);
    void (*get_data)(Encoder_t *encoder, EncoderData_t *data);
};

/** @brief 初始化编码器对象 */
bool EncoderInit(Encoder_t *encoder, const EncoderInitConfig_t *config);
/** @brief 启动编码器计数 */
bool EncoderStart(Encoder_t *encoder);
/** @brief 停止编码器计数 */
bool EncoderStop(Encoder_t *encoder);
/** @brief 更新编码器状态 */
void EncoderUpdate(Encoder_t *encoder, float dt_s);
/** @brief 复位编码器计数 */
void EncoderReset(Encoder_t *encoder);
/** @brief 获取累计计数 */
int32_t EncoderGetCount(Encoder_t *encoder);
/** @brief 获取当前方向 */
EncoderDirection_t EncoderGetDirection(Encoder_t *encoder);
/** @brief 获取全部运行数据 */
void EncoderGetData(Encoder_t *encoder, EncoderData_t *data);

#define ENCODER_OBJECT_DEFAULT                       \
        .init          = EncoderInit,                \
        .start         = EncoderStart,               \
        .stop          = EncoderStop,                \
        .update        = EncoderUpdate,              \
        .reset         = EncoderReset,               \
        .get_count     = EncoderGetCount,            \
        .get_direction = EncoderGetDirection,        \
        .get_data      = EncoderGetData
