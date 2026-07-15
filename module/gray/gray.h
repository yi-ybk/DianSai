/**
 * @file    gray.h
 * @brief   数字灰度传感器模块头文件
 * @details 基于bsp_gpio封装任意路数字灰度输入，支持配置黑线有效电平和对象式访问接口。
 */
#pragma once

#include "bsp_gpio.h"
#include <stdbool.h>
#include <stdint.h>

#define GRAY_MAX_CHANNELS 16U /**< 单个灰度传感器对象支持的最大通道数 */

/** @brief 灰度传感器逻辑颜色 */
typedef enum
{
    GRAY_COLOR_WHITE = 0, /**< 白色背景 */
    GRAY_COLOR_BLACK,     /**< 黑色线条 */
} GrayColor_t;

/** @brief 单路灰度输入配置 */
typedef struct
{
    GPIO_TypeDef *GPIOx; /**< GPIO端口 */
    uint16_t GPIO_Pin;   /**< GPIO引脚 */
} GrayChannelConfig_t;

/** @brief 灰度传感器初始化配置 */
typedef struct
{
    const GrayChannelConfig_t *channels; /**< 通道配置数组，数组顺序对应位图bit顺序 */
    uint8_t channel_count;               /**< 实际使用的通道数 */
    GPIO_PinState black_state;           /**< 检测到黑线时的GPIO电平，另一电平表示白色 */
} GrayInitConfig_t;

/** @brief 灰度传感器运行数据 */
typedef struct
{
    uint32_t black_mask;   /**< 黑线位图，bit为1表示对应通道检测到黑线 */
    uint32_t white_mask;   /**< 白色位图，bit为1表示对应通道检测到白色 */
    uint8_t channel_count; /**< 当前通道数 */
    uint32_t update_count; /**< 采样更新次数 */
} GrayData_t;

typedef struct Gray Gray_t;

/** @brief 灰度传感器对象 */
struct Gray
{
    GPIOInstance *gpio[GRAY_MAX_CHANNELS]; /**< 底层GPIO对象数组 */
    bool initialized;                      /**< 初始化标识 */
    GrayInitConfig_t init_config;          /**< 初始化配置缓存 */
    GrayData_t data;                       /**< 当前运行数据 */

    bool (*init)(Gray_t *gray, const GrayInitConfig_t *config);
    void (*update)(Gray_t *gray);
    GrayColor_t (*read_channel)(Gray_t *gray, uint8_t channel);
    uint32_t (*get_black_mask)(Gray_t *gray);
    void (*get_data)(Gray_t *gray, GrayData_t *data);
};

/** @brief 初始化灰度传感器对象 */
bool GrayInit(Gray_t *gray, const GrayInitConfig_t *config);
/** @brief 更新全部灰度输入 */
void GrayUpdate(Gray_t *gray);
/** @brief 读取指定通道的逻辑颜色 */
GrayColor_t GrayReadChannel(Gray_t *gray, uint8_t channel);
/** @brief 获取黑线位图 */
uint32_t GrayGetBlackMask(Gray_t *gray);
/** @brief 获取灰度传感器运行数据 */
void GrayGetData(Gray_t *gray, GrayData_t *data);

#define GRAY_OBJECT_DEFAULT              \
        .init           = GrayInit,      \
        .update         = GrayUpdate,    \
        .read_channel   = GrayReadChannel, \
        .get_black_mask = GrayGetBlackMask, \
        .get_data       = GrayGetData
