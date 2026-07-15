/**
 * @file    gray.c
 * @brief   数字灰度传感器模块实现
 * @details 通过GPIO BSP读取多路数字灰度信号，并转换为黑线位图、白色位图和单路逻辑颜色。
 */
#include "gray.h"
#include <string.h>

static void GrayBindMethods(Gray_t *gray);
static bool GrayConfigIsValid(const GrayInitConfig_t *config);
static bool GrayChannelConfigIsValid(const GrayChannelConfig_t *config);
static GrayColor_t GrayPinStateToColor(const Gray_t *gray, GPIO_PinState pin_state);

/**
 * @brief   初始化灰度传感器对象
 * @param   gray 灰度传感器对象指针
 * @param   config 灰度传感器初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool GrayInit(Gray_t *gray, const GrayInitConfig_t *config)
{
    GPIO_Init_Config_s gpio_config;
    uint8_t channel;

    if ((gray == NULL) || (config == NULL))
        return false;

    GrayBindMethods(gray);
    if (gray->initialized)
        return true;

    gray->init_config = *config;
    if (!GrayConfigIsValid(&gray->init_config))
        return false;

    memset(gray->gpio, 0, sizeof(gray->gpio));
    memset(&gray->data, 0, sizeof(gray->data));
    gray->data.channel_count = gray->init_config.channel_count;

    for (channel = 0U; channel < gray->init_config.channel_count; ++channel)
    {
        memset(&gpio_config, 0, sizeof(gpio_config));
        gpio_config.GPIOx = gray->init_config.channels[channel].GPIOx;
        gpio_config.GPIO_Pin = gray->init_config.channels[channel].GPIO_Pin;
        gpio_config.pin_state = GPIO_PIN_RESET;
        gpio_config.exti_mode = GPIO_EXTI_MODE_NONE;
        gpio_config.gpio_model_callback = NULL;
        gpio_config.id = gray;

        gray->gpio[channel] = GPIORegister(&gpio_config);
        if (gray->gpio[channel] == NULL)
            return false;
    }

    gray->initialized = true;
    GrayUpdate(gray);
    return true;
}

/**
 * @brief   更新全部灰度输入
 * @param   gray 灰度传感器对象指针
 */
void GrayUpdate(Gray_t *gray)
{
    GPIO_PinState pin_state;
    uint32_t channel_mask;
    uint8_t channel;

    if ((gray == NULL) || (!gray->initialized))
        return;

    gray->data.black_mask = 0U;
    gray->data.white_mask = 0U;

    for (channel = 0U; channel < gray->init_config.channel_count; ++channel)
    {
        pin_state = GPIORead(gray->gpio[channel]);
        channel_mask = 1UL << channel;
        if (GrayPinStateToColor(gray, pin_state) == GRAY_COLOR_BLACK)
            gray->data.black_mask |= channel_mask;
        else
            gray->data.white_mask |= channel_mask;
    }

    gray->data.update_count++;
}

/**
 * @brief   读取指定通道的逻辑颜色
 * @param   gray 灰度传感器对象指针
 * @param   channel 通道索引，取值范围为0到channel_count-1
 * @return  GrayColor_t 当前逻辑颜色，无效通道返回白色
 */
GrayColor_t GrayReadChannel(Gray_t *gray, uint8_t channel)
{
    if ((gray == NULL) || (!gray->initialized) ||
        (channel >= gray->init_config.channel_count))
    {
        return GRAY_COLOR_WHITE;
    }

    GrayUpdate(gray);
    return ((gray->data.black_mask & (1UL << channel)) != 0U) ?
           GRAY_COLOR_BLACK : GRAY_COLOR_WHITE;
}

/**
 * @brief   获取黑线位图
 * @param   gray 灰度传感器对象指针
 * @return  uint32_t 黑线位图，bit为1表示对应通道检测到黑线
 */
uint32_t GrayGetBlackMask(Gray_t *gray)
{
    if ((gray == NULL) || (!gray->initialized))
        return 0U;

    GrayUpdate(gray);
    return gray->data.black_mask;
}

/**
 * @brief   获取灰度传感器运行数据
 * @param   gray 灰度传感器对象指针
 * @param   data 数据输出目标
 */
void GrayGetData(Gray_t *gray, GrayData_t *data)
{
    if ((gray == NULL) || (data == NULL))
        return;

    if (gray->initialized)
        GrayUpdate(gray);

    memcpy(data, &gray->data, sizeof(*data));
}

/**
 * @brief   绑定灰度传感器对象方法
 * @param   gray 灰度传感器对象指针
 */
static void GrayBindMethods(Gray_t *gray)
{
    if (gray == NULL)
        return;

    gray->init = GrayInit;
    gray->update = GrayUpdate;
    gray->read_channel = GrayReadChannel;
    gray->get_black_mask = GrayGetBlackMask;
    gray->get_data = GrayGetData;
}

/**
 * @brief   校验灰度传感器初始化配置
 * @param   config 灰度传感器初始化配置
 * @return  bool 合法返回true，否则返回false
 */
static bool GrayConfigIsValid(const GrayInitConfig_t *config)
{
    uint8_t channel;

    if ((config == NULL) || (config->channels == NULL) ||
        (config->channel_count == 0U) ||
        (config->channel_count > GRAY_MAX_CHANNELS))
    {
        return false;
    }

    if ((config->black_state != GPIO_PIN_SET) &&
        (config->black_state != GPIO_PIN_RESET))
    {
        return false;
    }

    for (channel = 0U; channel < config->channel_count; ++channel)
    {
        if (!GrayChannelConfigIsValid(&config->channels[channel]))
            return false;
    }

    return true;
}

/**
 * @brief   校验单路灰度输入配置
 * @param   config 单路灰度输入配置
 * @return  bool 合法返回true，否则返回false
 */
static bool GrayChannelConfigIsValid(const GrayChannelConfig_t *config)
{
    return ((config != NULL) &&
            (config->GPIOx != NULL) &&
            (config->GPIO_Pin != 0U));
}

/**
 * @brief   将GPIO实际电平转换为灰度逻辑颜色
 * @param   gray 灰度传感器对象指针
 * @param   pin_state GPIO实际电平
 * @return  GrayColor_t 灰度逻辑颜色
 */
static GrayColor_t GrayPinStateToColor(const Gray_t *gray, GPIO_PinState pin_state)
{
    if (gray == NULL)
        return GRAY_COLOR_WHITE;

    return (pin_state == gray->init_config.black_state) ?
           GRAY_COLOR_BLACK : GRAY_COLOR_WHITE;
}
