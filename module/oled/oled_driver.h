/**
 * @file    oled_driver.h
 * @brief   SSD1306 OLED模块驱动头文件
 * @details 封装0.96寸I2C SSD1306屏幕初始化、显存刷新和基础绘图接口。
 */
#pragma once

#include "bsp_iic.h"
#include <stdbool.h>
#include <stdint.h>

#define OLED_SSD1306_DEFAULT_ADDR 0x3CU
#define OLED_SSD1306_WIDTH        128U
#define OLED_SSD1306_HEIGHT       64U
#define OLED_SSD1306_BUFFER_SIZE  ((OLED_SSD1306_WIDTH * OLED_SSD1306_HEIGHT) / 8U)

typedef enum
{
    OLED_COLOR_BLACK = 0,
    OLED_COLOR_WHITE,
    OLED_COLOR_INVERT,
} OledColor_t;

typedef struct
{
    I2C_HandleTypeDef *i2c_handle; // 硬件I2C句柄,软件IIC模式可为NULL
    IIC_Bus_Mode_e iic_bus_mode;   // IIC总线类型
    IIC_Soft_Config_s soft_iic;    // 软件IIC GPIO配置
    uint8_t dev_address;           // SSD1306 7位地址,默认0x3C
    uint16_t width;                // 屏幕宽度,为0时默认128
    uint16_t height;               // 屏幕高度,为0时默认64
    bool rotate_180;               // 是否旋转180度
    bool inverted;                 // 是否反色显示
} OledInitConfig_t;

typedef struct
{
    uint16_t width;
    uint16_t height;
    uint16_t cursor_x;
    uint16_t cursor_y;
    bool display_on;
} OledData_t;

typedef struct Oled Oled_t;

struct Oled
{
    IICInstance *iic;
    bool initialized;
    OledInitConfig_t init_config;
    OledData_t data;
    uint8_t buffer[OLED_SSD1306_BUFFER_SIZE];

    bool (*init)(Oled_t *oled, const OledInitConfig_t *config);
    bool (*refresh)(Oled_t *oled);
    void (*clear)(Oled_t *oled);
    void (*fill)(Oled_t *oled, OledColor_t color);
    void (*draw_pixel)(Oled_t *oled, uint16_t x, uint16_t y, OledColor_t color);
    void (*draw_rect)(Oled_t *oled, uint16_t x, uint16_t y, uint16_t width, uint16_t height, OledColor_t color);
    void (*draw_char)(Oled_t *oled, uint16_t x, uint16_t y, char ch, OledColor_t color);
    void (*draw_string)(Oled_t *oled, uint16_t x, uint16_t y, const char *str, OledColor_t color);
    bool (*show_string)(Oled_t *oled, uint16_t x, uint16_t y, const char *str, OledColor_t color);
    bool (*show_int)(Oled_t *oled, uint16_t x, uint16_t y, int32_t value, OledColor_t color);
    bool (*show_float)(Oled_t *oled, uint16_t x, uint16_t y, float value, uint8_t decimals, OledColor_t color);
    bool (*set_display)(Oled_t *oled, bool on);
    bool (*set_inverted)(Oled_t *oled, bool inverted);
    void (*get_data)(Oled_t *oled, OledData_t *data);
};

/** @brief 初始化OLED对象 */
bool OledInit(Oled_t *oled, const OledInitConfig_t *config);
/** @brief 将显存内容刷新到OLED屏幕 */
bool OledRefresh(Oled_t *oled);
/** @brief 清空OLED显存 */
void OledClear(Oled_t *oled);
/** @brief 填充OLED显存 */
void OledFill(Oled_t *oled, OledColor_t color);
/** @brief 在显存中绘制一个像素点 */
void OledDrawPixel(Oled_t *oled, uint16_t x, uint16_t y, OledColor_t color);
/** @brief 在显存中绘制实心矩形 */
void OledDrawRect(Oled_t *oled, uint16_t x, uint16_t y, uint16_t width, uint16_t height, OledColor_t color);
/** @brief 在显存中绘制一个ASCII字符 */
void OledDrawChar(Oled_t *oled, uint16_t x, uint16_t y, char ch, OledColor_t color);
/** @brief 在显存中绘制ASCII字符串,需要手动调用OledRefresh刷新 */
void OledDrawString(Oled_t *oled, uint16_t x, uint16_t y, const char *str, OledColor_t color);
/** @brief 显示ASCII字符串,函数内部会自动刷新屏幕 */
bool OledShowString(Oled_t *oled, uint16_t x, uint16_t y, const char *str, OledColor_t color);
/** @brief 显示有符号整数,支持正负数 */
bool OledShowInt(Oled_t *oled, uint16_t x, uint16_t y, int32_t value, OledColor_t color);
/** @brief 显示浮点数,decimals为小数位数,支持正负数 */
bool OledShowFloat(Oled_t *oled, uint16_t x, uint16_t y, float value, uint8_t decimals, OledColor_t color);
/** @brief 开启或关闭OLED显示 */
bool OledSetDisplay(Oled_t *oled, bool on);
/** @brief 设置OLED是否反色显示 */
bool OledSetInverted(Oled_t *oled, bool inverted);
/** @brief 获取OLED运行数据 */
void OledGetData(Oled_t *oled, OledData_t *data);

#define OLED_OBJECT_DEFAULT                \
        .init         = OledInit,          \
        .refresh      = OledRefresh,       \
        .clear        = OledClear,         \
        .fill         = OledFill,          \
        .draw_pixel   = OledDrawPixel,     \
        .draw_rect    = OledDrawRect,      \
        .draw_char    = OledDrawChar,      \
        .draw_string  = OledDrawString,    \
        .show_string  = OledShowString,    \
        .show_int     = OledShowInt,       \
        .show_float   = OledShowFloat,     \
        .set_display  = OledSetDisplay,    \
        .set_inverted = OledSetInverted,   \
        .get_data     = OledGetData
