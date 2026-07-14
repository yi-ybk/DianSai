/**
 * @file    oled_driver.c
 * @brief   SSD1306 OLED模块驱动实现
 * @details 通过bsp_iic封装0.96寸I2C SSD1306屏幕的命令、显存和基础绘图操作。
 */
#include "oled_driver.h"
#include <string.h>

#define OLED_SSD1306_CMD_CONTROL    0x00U
#define OLED_SSD1306_DATA_CONTROL   0x40U
#define OLED_SSD1306_PAGE_HEIGHT    8U
#define OLED_SSD1306_I2C_CHUNK_SIZE 16U
#define OLED_NUMBER_BUFFER_LEN      24U
#define OLED_FLOAT_DECIMAL_MAX      6U

static const uint8_t oled_font_5x7[][5] =
{
    {0x00, 0x00, 0x00, 0x00, 0x00}, /*   */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* ! */
    {0x00, 0x07, 0x00, 0x07, 0x00}, /* " */
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, /* # */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* $ */
    {0x23, 0x13, 0x08, 0x64, 0x62}, /* % */
    {0x36, 0x49, 0x55, 0x22, 0x50}, /* & */
    {0x00, 0x05, 0x03, 0x00, 0x00}, /* ' */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* ( */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* ) */
    {0x14, 0x08, 0x3E, 0x08, 0x14}, /* * */
    {0x08, 0x08, 0x3E, 0x08, 0x08}, /* + */
    {0x00, 0x50, 0x30, 0x00, 0x00}, /* , */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* - */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* . */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* / */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2 */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5 */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 9 */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* : */
    {0x00, 0x56, 0x36, 0x00, 0x00}, /* ; */
    {0x08, 0x14, 0x22, 0x41, 0x00}, /* < */
    {0x14, 0x14, 0x14, 0x14, 0x14}, /* = */
    {0x00, 0x41, 0x22, 0x14, 0x08}, /* > */
    {0x02, 0x01, 0x51, 0x09, 0x06}, /* ? */
    {0x32, 0x49, 0x79, 0x41, 0x3E}, /* @ */
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* A */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* Z */
    {0x00, 0x7F, 0x41, 0x41, 0x00}, /* [ */
    {0x02, 0x04, 0x08, 0x10, 0x20}, /* \ */
    {0x00, 0x41, 0x41, 0x7F, 0x00}, /* ] */
    {0x04, 0x02, 0x01, 0x02, 0x04}, /* ^ */
    {0x40, 0x40, 0x40, 0x40, 0x40}, /* _ */
    {0x00, 0x01, 0x02, 0x04, 0x00}, /* ` */
    {0x20, 0x54, 0x54, 0x54, 0x78}, /* a */
    {0x7F, 0x48, 0x44, 0x44, 0x38}, /* b */
    {0x38, 0x44, 0x44, 0x44, 0x20}, /* c */
    {0x38, 0x44, 0x44, 0x48, 0x7F}, /* d */
    {0x38, 0x54, 0x54, 0x54, 0x18}, /* e */
    {0x08, 0x7E, 0x09, 0x01, 0x02}, /* f */
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, /* g */
    {0x7F, 0x08, 0x04, 0x04, 0x78}, /* h */
    {0x00, 0x44, 0x7D, 0x40, 0x00}, /* i */
    {0x20, 0x40, 0x44, 0x3D, 0x00}, /* j */
    {0x7F, 0x10, 0x28, 0x44, 0x00}, /* k */
    {0x00, 0x41, 0x7F, 0x40, 0x00}, /* l */
    {0x7C, 0x04, 0x18, 0x04, 0x78}, /* m */
    {0x7C, 0x08, 0x04, 0x04, 0x78}, /* n */
    {0x38, 0x44, 0x44, 0x44, 0x38}, /* o */
    {0x7C, 0x14, 0x14, 0x14, 0x08}, /* p */
    {0x08, 0x14, 0x14, 0x18, 0x7C}, /* q */
    {0x7C, 0x08, 0x04, 0x04, 0x08}, /* r */
    {0x48, 0x54, 0x54, 0x54, 0x20}, /* s */
    {0x04, 0x3F, 0x44, 0x40, 0x20}, /* t */
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, /* u */
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, /* v */
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, /* w */
    {0x44, 0x28, 0x10, 0x28, 0x44}, /* x */
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, /* y */
    {0x44, 0x64, 0x54, 0x4C, 0x44}, /* z */
    {0x00, 0x08, 0x36, 0x41, 0x00}, /* { */
    {0x00, 0x00, 0x7F, 0x00, 0x00}, /* | */
    {0x00, 0x41, 0x36, 0x08, 0x00}, /* } */
    {0x08, 0x04, 0x08, 0x10, 0x08}, /* ~ */
};

static void OledBindMethods(Oled_t *oled);
static bool OledConfigIsValid(const OledInitConfig_t *config);
static bool OledWriteCommand(Oled_t *oled, uint8_t command);
static bool OledWriteData(Oled_t *oled, const uint8_t *data, uint16_t size);
static uint8_t OledGetFontColumn(char ch, uint8_t column);
static char *OledAppendUnsigned(char *buffer, char *end, uint32_t value, uint8_t min_width);
static void OledFormatInt(char *buffer, uint16_t buffer_len, int32_t value);
static void OledFormatFloat(char *buffer, uint16_t buffer_len, float value, uint8_t decimals);
static uint32_t OledPow10(uint8_t exponent);

bool OledInit(Oled_t *oled, const OledInitConfig_t *config)
{
    IIC_Init_Config_s iic_config;
    uint16_t width;
    uint16_t height;

    if ((oled == NULL) || (!OledConfigIsValid(config)))
        return false;

    OledBindMethods(oled);
    if (oled->initialized)
        return true;

    width = (config->width == 0U) ? OLED_SSD1306_WIDTH : config->width;
    height = (config->height == 0U) ? OLED_SSD1306_HEIGHT : config->height;

    memset(oled, 0, sizeof(*oled));
    OledBindMethods(oled);
    oled->init_config = *config;
    oled->init_config.width = width;
    oled->init_config.height = height;
    if (oled->init_config.dev_address == 0U)
        oled->init_config.dev_address = OLED_SSD1306_DEFAULT_ADDR;

    memset(&iic_config, 0, sizeof(iic_config));
    iic_config.handle = oled->init_config.i2c_handle;
    iic_config.dev_address = oled->init_config.dev_address;
    iic_config.bus_mode = oled->init_config.iic_bus_mode;
    iic_config.work_mode = IIC_BLOCK_MODE;
    iic_config.soft_config = oled->init_config.soft_iic;
    iic_config.id = oled;
    oled->iic = IICRegister(&iic_config);
    if (oled->iic == NULL)
        return false;

    oled->data.width = width;
    oled->data.height = height;

    (void)OledWriteCommand(oled, 0xAEU);
    (void)OledWriteCommand(oled, 0x20U);
    (void)OledWriteCommand(oled, 0x00U);
    (void)OledWriteCommand(oled, 0xB0U);
    (void)OledWriteCommand(oled, oled->init_config.rotate_180 ? 0xC0U : 0xC8U);
    (void)OledWriteCommand(oled, 0x00U);
    (void)OledWriteCommand(oled, 0x10U);
    (void)OledWriteCommand(oled, 0x40U);
    (void)OledWriteCommand(oled, 0x81U);
    (void)OledWriteCommand(oled, 0x7FU);
    (void)OledWriteCommand(oled, oled->init_config.rotate_180 ? 0xA0U : 0xA1U);
    (void)OledWriteCommand(oled, oled->init_config.inverted ? 0xA7U : 0xA6U);
    (void)OledWriteCommand(oled, 0xA8U);
    (void)OledWriteCommand(oled, (uint8_t)(height - 1U));
    (void)OledWriteCommand(oled, 0xA4U);
    (void)OledWriteCommand(oled, 0xD3U);
    (void)OledWriteCommand(oled, 0x00U);
    (void)OledWriteCommand(oled, 0xD5U);
    (void)OledWriteCommand(oled, 0x80U);
    (void)OledWriteCommand(oled, 0xD9U);
    (void)OledWriteCommand(oled, 0xF1U);
    (void)OledWriteCommand(oled, 0xDAU);
    (void)OledWriteCommand(oled, (height == 64U) ? 0x12U : 0x02U);
    (void)OledWriteCommand(oled, 0xDBU);
    (void)OledWriteCommand(oled, 0x40U);
    (void)OledWriteCommand(oled, 0x8DU);
    (void)OledWriteCommand(oled, 0x14U);

    oled->initialized = true;
    OledClear(oled);

    return OledSetDisplay(oled, true);
}

bool OledRefresh(Oled_t *oled)
{
    uint8_t pages;
    uint8_t page_mask;

    if ((oled == NULL) || (!oled->initialized))
        return false;

    pages = (uint8_t)(oled->data.height / OLED_SSD1306_PAGE_HEIGHT);
    for (uint8_t page = 0; page < pages; page++)
    {
        page_mask = (uint8_t)(1U << page);
        if ((oled->dirty_pages & page_mask) == 0U)
            continue;

        if ((!OledWriteCommand(oled, (uint8_t)(0xB0U + page))) ||
            (!OledWriteCommand(oled, 0x00U)) ||
            (!OledWriteCommand(oled, 0x10U)) ||
            (!OledWriteData(oled,
                            &oled->buffer[(uint16_t)page * oled->data.width],
                            oled->data.width)))
        {
            return false;
        }

        oled->dirty_pages &= (uint8_t)(~page_mask);
    }

    return true;
}

bool OledRefreshAll(Oled_t *oled)
{
    uint8_t pages;

    if ((oled == NULL) || (!oled->initialized))
        return false;

    pages = (uint8_t)(oled->data.height / OLED_SSD1306_PAGE_HEIGHT);
    oled->dirty_pages = (pages >= 8U) ? 0xFFU : (uint8_t)((1U << pages) - 1U);

    return OledRefresh(oled);
}

void OledClear(Oled_t *oled)
{
    if (oled == NULL)
        return;

    OledFill(oled, OLED_COLOR_BLACK);
    if (oled->initialized)
        (void)OledRefreshAll(oled);
}

void OledFill(Oled_t *oled, OledColor_t color)
{
    uint8_t fill_value;
    uint8_t pages;
    uint8_t old_value;
    uint8_t new_value;
    uint16_t index;
    bool page_changed;

    if (oled == NULL)
        return;

    fill_value = (color == OLED_COLOR_WHITE) ? 0xFFU : 0x00U;
    pages = (uint8_t)(oled->data.height / OLED_SSD1306_PAGE_HEIGHT);
    for (uint8_t page = 0; page < pages; page++)
    {
        page_changed = false;
        for (uint16_t x = 0; x < oled->data.width; x++)
        {
            index = ((uint16_t)page * oled->data.width) + x;
            old_value = oled->buffer[index];
            new_value = (color == OLED_COLOR_INVERT) ?
                        (uint8_t)(old_value ^ 0xFFU) : fill_value;
            if (new_value != old_value)
            {
                oled->buffer[index] = new_value;
                page_changed = true;
            }
        }

        if (page_changed)
            oled->dirty_pages |= (uint8_t)(1U << page);
    }
}

void OledDrawPixel(Oled_t *oled, uint16_t x, uint16_t y, OledColor_t color)
{
    uint16_t index;
    uint8_t mask;
    uint8_t old_value;

    if ((oled == NULL) || (x >= oled->data.width) || (y >= oled->data.height))
        return;

    index = x + ((y / OLED_SSD1306_PAGE_HEIGHT) * oled->data.width);
    mask = (uint8_t)(1U << (y % OLED_SSD1306_PAGE_HEIGHT));
    old_value = oled->buffer[index];

    if (color == OLED_COLOR_WHITE)
        oled->buffer[index] |= mask;
    else if (color == OLED_COLOR_BLACK)
        oled->buffer[index] &= (uint8_t)(~mask);
    else
        oled->buffer[index] ^= mask;

    if (oled->buffer[index] != old_value)
        oled->dirty_pages |= (uint8_t)(1U << (y / OLED_SSD1306_PAGE_HEIGHT));
}

void OledDrawRect(Oled_t *oled, uint16_t x, uint16_t y, uint16_t width, uint16_t height, OledColor_t color)
{
    if ((oled == NULL) || (width == 0U) || (height == 0U))
        return;

    for (uint16_t row = 0; row < height; row++)
    {
        for (uint16_t col = 0; col < width; col++)
            OledDrawPixel(oled, x + col, y + row, color);
    }
}

void OledDrawChar(Oled_t *oled, uint16_t x, uint16_t y, char ch, OledColor_t color)
{
    uint16_t pixel_x;
    uint16_t pixel_y;
    uint8_t column_data;

    if (oled == NULL)
        return;

    if ((x >= (oled->data.width / OLED_ASCII_CHAR_WIDTH)) ||
        (y >= (oled->data.height / OLED_ASCII_CHAR_HEIGHT)))
    {
        return;
    }

    pixel_x = (uint16_t)(x * OLED_ASCII_CHAR_WIDTH);
    pixel_y = (uint16_t)(y * OLED_ASCII_CHAR_HEIGHT);

    for (uint8_t col = 0; col < OLED_ASCII_CHAR_WIDTH; col++)
    {
        column_data = (col < 5U) ? OledGetFontColumn(ch, col) : 0x00U;
        for (uint8_t row = 0; row < OLED_ASCII_CHAR_HEIGHT; row++)
        {
            if ((column_data & (1U << row)) != 0U)
                OledDrawPixel(oled, pixel_x + col, pixel_y + row, color);
            else if (color == OLED_COLOR_BLACK)
                OledDrawPixel(oled, pixel_x + col, pixel_y + row, OLED_COLOR_WHITE);
            else
                OledDrawPixel(oled, pixel_x + col, pixel_y + row, OLED_COLOR_BLACK);
        }
    }
}

void OledDrawString(Oled_t *oled, uint16_t x, uint16_t y, const char *str, OledColor_t color)
{
    uint16_t cursor_x;
    uint16_t cursor_y;
    uint16_t text_columns;
    uint16_t text_rows;

    if ((oled == NULL) || (str == NULL))
        return;

    text_columns = (uint16_t)(oled->data.width / OLED_ASCII_CHAR_WIDTH);
    text_rows = (uint16_t)(oled->data.height / OLED_ASCII_CHAR_HEIGHT);
    if ((x >= text_columns) || (y >= text_rows))
        return;

    cursor_x = x;
    cursor_y = y;
    while (*str != '\0')
    {
        if (*str == '\n')
        {
            cursor_x = x;
            cursor_y++;
            str++;
            continue;
        }

        if (cursor_x >= text_columns)
        {
            cursor_x = x;
            cursor_y++;
        }

        if (cursor_y >= text_rows)
            break;

        OledDrawChar(oled, cursor_x, cursor_y, *str, color);
        cursor_x++;
        str++;
    }

    oled->data.cursor_x = cursor_x;
    oled->data.cursor_y = cursor_y;
}

void OledDrawInt(Oled_t *oled, uint16_t x, uint16_t y, int32_t value, OledColor_t color)
{
    char buffer[OLED_NUMBER_BUFFER_LEN];

    if (oled == NULL)
        return;

    OledFormatInt(buffer, sizeof(buffer), value);
    OledDrawString(oled, x, y, buffer, color);
}

void OledDrawFloat(Oled_t *oled,
                   uint16_t x,
                   uint16_t y,
                   float value,
                   uint8_t decimals,
                   OledColor_t color)
{
    char buffer[OLED_NUMBER_BUFFER_LEN];

    if (oled == NULL)
        return;

    OledFormatFloat(buffer, sizeof(buffer), value, decimals);
    OledDrawString(oled, x, y, buffer, color);
}

/**
 * @brief 显示字符串并刷新屏幕
 *
 * @param oled OLED对象指针
 * @param x 首字符列号
 * @param y 起始文本行号
 * @param str 待显示字符串
 * @param color 显示颜色
 * @return bool 成功返回true
 */
bool OledShowString(Oled_t *oled, uint16_t x, uint16_t y, const char *str, OledColor_t color)
{
    if ((oled == NULL) || (str == NULL))
        return false;

    OledDrawString(oled, x, y, str, color);
    return OledRefresh(oled);
}

/**
 * @brief 显示有符号整数并刷新屏幕
 *
 * @param oled OLED对象指针
 * @param x 首字符列号
 * @param y 起始文本行号
 * @param value 待显示整数,支持正负数
 * @param color 显示颜色
 * @return bool 成功返回true
 */
bool OledShowInt(Oled_t *oled, uint16_t x, uint16_t y, int32_t value, OledColor_t color)
{
    if (oled == NULL)
        return false;

    OledDrawInt(oled, x, y, value, color);
    return OledRefresh(oled);
}

/**
 * @brief 显示浮点数并刷新屏幕
 *
 * @param oled OLED对象指针
 * @param x 首字符列号
 * @param y 起始文本行号
 * @param value 待显示浮点数,支持正负数
 * @param decimals 小数位数,最大OLED_FLOAT_DECIMAL_MAX
 * @param color 显示颜色
 * @return bool 成功返回true
 */
bool OledShowFloat(Oled_t *oled, uint16_t x, uint16_t y, float value, uint8_t decimals, OledColor_t color)
{
    if (oled == NULL)
        return false;

    OledDrawFloat(oled, x, y, value, decimals, color);
    return OledRefresh(oled);
}

bool OledSetDisplay(Oled_t *oled, bool on)
{
    if (oled == NULL)
        return false;

    if (!OledWriteCommand(oled, on ? 0xAFU : 0xAEU))
        return false;

    oled->data.display_on = on;
    return true;
}

bool OledSetInverted(Oled_t *oled, bool inverted)
{
    if (oled == NULL)
        return false;

    if (!OledWriteCommand(oled, inverted ? 0xA7U : 0xA6U))
        return false;

    oled->init_config.inverted = inverted;
    return true;
}

void OledGetData(Oled_t *oled, OledData_t *data)
{
    if ((oled == NULL) || (data == NULL))
        return;

    memcpy(data, &oled->data, sizeof(*data));
}

static void OledBindMethods(Oled_t *oled)
{
    if (oled == NULL)
        return;

    oled->init          = OledInit;
    oled->refresh       = OledRefresh;
    oled->refresh_all   = OledRefreshAll;
    oled->clear         = OledClear;
    oled->fill          = OledFill;
    oled->draw_pixel    = OledDrawPixel;
    oled->draw_rect     = OledDrawRect;
    oled->draw_char     = OledDrawChar;
    oled->draw_string   = OledDrawString;
    oled->draw_int      = OledDrawInt;
    oled->draw_float    = OledDrawFloat;
    oled->show_string   = OledShowString;
    oled->show_int      = OledShowInt;
    oled->show_float    = OledShowFloat;
    oled->set_display   = OledSetDisplay;
    oled->set_inverted  = OledSetInverted;
    oled->get_data      = OledGetData;
}

static bool OledConfigIsValid(const OledInitConfig_t *config)
{
    uint16_t width;
    uint16_t height;

    if (config == NULL)
        return false;

    if ((config->iic_bus_mode == IIC_BUS_HARDWARE) && (config->i2c_handle == NULL))
        return false;

    if ((config->iic_bus_mode == IIC_BUS_SOFTWARE) &&
        ((config->soft_iic.scl.GPIOx == NULL) ||
         (config->soft_iic.sda.GPIOx == NULL) ||
         (config->soft_iic.scl.GPIO_Pin == 0U) ||
         (config->soft_iic.sda.GPIO_Pin == 0U)))
    {
        return false;
    }

    if ((config->iic_bus_mode != IIC_BUS_HARDWARE) &&
        (config->iic_bus_mode != IIC_BUS_SOFTWARE))
    {
        return false;
    }

    width = (config->width == 0U) ? OLED_SSD1306_WIDTH : config->width;
    height = (config->height == 0U) ? OLED_SSD1306_HEIGHT : config->height;

    if ((width == 0U) || (width > OLED_SSD1306_WIDTH) ||
        (height == 0U) || (height > OLED_SSD1306_HEIGHT) ||
        ((height % OLED_SSD1306_PAGE_HEIGHT) != 0U))
    {
        return false;
    }

    return true;
}

static bool OledWriteCommand(Oled_t *oled, uint8_t command)
{
    uint8_t buffer[2];

    if ((oled == NULL) || (oled->iic == NULL))
        return false;

    buffer[0] = OLED_SSD1306_CMD_CONTROL;
    buffer[1] = command;
    IICTransmit(oled->iic, buffer, sizeof(buffer), IIC_SEQ_RELEASE);

    return true;
}

static bool OledWriteData(Oled_t *oled, const uint8_t *data, uint16_t size)
{
    uint8_t packet[OLED_SSD1306_I2C_CHUNK_SIZE + 1U];
    uint16_t offset;
    uint16_t chunk_size;

    if ((oled == NULL) || (oled->iic == NULL) || (data == NULL))
        return false;

    offset = 0U;
    while (offset < size)
    {
        chunk_size = (uint16_t)(size - offset);
        if (chunk_size > OLED_SSD1306_I2C_CHUNK_SIZE)
            chunk_size = OLED_SSD1306_I2C_CHUNK_SIZE;

        packet[0] = OLED_SSD1306_DATA_CONTROL;
        memcpy(&packet[1], &data[offset], chunk_size);
        IICTransmit(oled->iic, packet, (uint16_t)(chunk_size + 1U), IIC_SEQ_RELEASE);
        offset = (uint16_t)(offset + chunk_size);
    }

    return true;
}

static uint8_t OledGetFontColumn(char ch, uint8_t column)
{
    if ((ch < ' ') || (ch > '~') || (column >= 5U))
        return oled_font_5x7['?' - ' '][column];

    return oled_font_5x7[ch - ' '][column];
}

/**
 * @brief 追加无符号整数到字符串缓冲区
 *
 * @param buffer 当前写入位置
 * @param end 缓冲区最后一个可写字符位置
 * @param value 待写入数值
 * @param min_width 最小位宽,不足时在左侧补0
 * @return char* 新的写入位置
 */
static char *OledAppendUnsigned(char *buffer, char *end, uint32_t value, uint8_t min_width)
{
    char temp[10];
    uint8_t len = 0U;

    if ((buffer == NULL) || (end == NULL) || (buffer > end))
        return buffer;

    do
    {
        temp[len++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (len < sizeof(temp)));

    while ((len < min_width) && (len < sizeof(temp)))
        temp[len++] = '0';

    while ((len > 0U) && (buffer < end))
        *buffer++ = temp[--len];

    *buffer = '\0';
    return buffer;
}

/**
 * @brief 格式化有符号整数
 *
 * @param buffer 输出缓冲区
 * @param buffer_len 输出缓冲区长度
 * @param value 待格式化整数
 */
static void OledFormatInt(char *buffer, uint16_t buffer_len, int32_t value)
{
    char *cursor;
    char *end;
    uint32_t magnitude;

    if ((buffer == NULL) || (buffer_len == 0U))
        return;

    buffer[0] = '\0';
    cursor = buffer;
    end    = buffer + buffer_len - 1U;

    if (value < 0)
    {
        if (cursor < end)
            *cursor++ = '-';
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    (void)OledAppendUnsigned(cursor, end, magnitude, 0U);
}

/**
 * @brief 格式化浮点数
 *
 * @param buffer 输出缓冲区
 * @param buffer_len 输出缓冲区长度
 * @param value 待格式化浮点数
 * @param decimals 小数位数
 */
static void OledFormatFloat(char *buffer, uint16_t buffer_len, float value, uint8_t decimals)
{
    char *cursor;
    char *end;
    uint32_t scale;
    uint32_t integer_part;
    uint32_t fraction_part;
    float fraction;

    if ((buffer == NULL) || (buffer_len == 0U))
        return;

    buffer[0] = '\0';
    if (decimals > OLED_FLOAT_DECIMAL_MAX)
        decimals = OLED_FLOAT_DECIMAL_MAX;

    cursor = buffer;
    end    = buffer + buffer_len - 1U;

    if (value != value)
    {
        if (buffer_len >= 4U)
        {
            buffer[0] = 'n';
            buffer[1] = 'a';
            buffer[2] = 'n';
            buffer[3] = '\0';
        }
        return;
    }

    if (value < 0.0f)
    {
        if (cursor < end)
            *cursor++ = '-';
        value = -value;
    }

    if (value > 4294967040.0f)
    {
        if (buffer_len >= 4U)
        {
            buffer[0] = 'o';
            buffer[1] = 'v';
            buffer[2] = 'f';
            buffer[3] = '\0';
        }
        return;
    }

    scale         = OledPow10(decimals);
    integer_part  = (uint32_t)value;
    fraction      = (value - (float)integer_part) * (float)scale + 0.5f;
    fraction_part = (uint32_t)fraction;
    if (fraction_part >= scale)
    {
        integer_part++;
        fraction_part -= scale;
    }

    cursor = OledAppendUnsigned(cursor, end, integer_part, 0U);
    if ((decimals > 0U) && (cursor < end))
    {
        *cursor++ = '.';
        *cursor = '\0';
        (void)OledAppendUnsigned(cursor, end, fraction_part, decimals);
    }
}

/**
 * @brief 计算10的整数次幂
 *
 * @param exponent 指数
 * @return uint32_t 10^exponent
 */
static uint32_t OledPow10(uint8_t exponent)
{
    uint32_t value = 1U;

    while (exponent > 0U)
    {
        value *= 10U;
        exponent--;
    }

    return value;
}
