#include "bsp_iic.h"
#include "bsp_delay.h"
#include "string.h"

static uint8_t idx = 0; // 配合中断以及初始化
static IICInstance iic_instance_pool[MX_IIC_SLAVE_CNT];
static IICInstance *iic_instance[MX_IIC_SLAVE_CNT] = {NULL};

static uint8_t IICConfigIsValid(const IIC_Init_Config_s *conf);
static uint8_t IICSoftConfigIsValid(const IIC_Soft_Config_s *conf);
static uint8_t IICInstanceMatches(const IICInstance *iic, const IIC_Init_Config_s *conf);
static void IICSoftInitGPIO(const IIC_Soft_Config_s *conf);
static void IICSoftInitGPIOPin(const IIC_GPIO_Config_s *gpio);
static void IICSoftEnableGPIOClock(GPIO_TypeDef *GPIOx);
static void IICSoftDelay(const IICInstance *iic);
static void IICSoftWriteSCL(const IICInstance *iic, GPIO_PinState state);
static void IICSoftWriteSDA(const IICInstance *iic, GPIO_PinState state);
static GPIO_PinState IICSoftReadSCL(const IICInstance *iic);
static GPIO_PinState IICSoftReadSDA(const IICInstance *iic);
static uint8_t IICSoftBusIsIdle(const IICInstance *iic);
static void IICSoftStart(const IICInstance *iic);
static void IICSoftStop(const IICInstance *iic);
static void IICSoftWriteBit(const IICInstance *iic, uint8_t bit);
static uint8_t IICSoftReadBit(const IICInstance *iic);
static uint8_t IICSoftWriteByte(const IICInstance *iic, uint8_t data);
static uint8_t IICSoftReadByte(const IICInstance *iic, uint8_t ack);
static uint8_t IICSoftTransmit(IICInstance *iic, const uint8_t *data, uint16_t size);
static uint8_t IICSoftReceive(IICInstance *iic, uint8_t *data, uint16_t size);
static uint8_t IICSoftAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag);
static uint8_t IICHardwareTransmit(IICInstance *iic, const uint8_t *data, uint16_t size);
static uint8_t IICHardwareReceive(IICInstance *iic, uint8_t *data, uint16_t size);
static uint8_t IICHardwareWaitComplete(I2C_Regs *i2c, uint32_t timeout_ms);

IICInstance *IICRegister(IIC_Init_Config_s *conf)
{
    IICInstance *instance;

    if (!IICConfigIsValid(conf))
        return NULL;

    for (uint8_t i = 0; i < idx; i++)
    {
        if ((iic_instance[i] != NULL) && IICInstanceMatches(iic_instance[i], conf))
            return iic_instance[i];
    }

    if (idx >= MX_IIC_SLAVE_CNT)
        return NULL;

    if (idx >= MX_IIC_SLAVE_CNT) // 超过最大实例数
        while (1)                // 酌情增加允许的实例上限,也有可能是内存泄漏
            ;
    // 申请到的空间未必是0, 所以需要手动初始化
    instance = &iic_instance_pool[idx];
    memset(instance, 0, sizeof(IICInstance));

    instance->dev_address = conf->dev_address;
    instance->callback = conf->callback;
    instance->work_mode = conf->work_mode;
    instance->bus_mode = conf->bus_mode;
    instance->handle = conf->handle;
    instance->id = conf->id;

    if (instance->bus_mode == IIC_BUS_SOFTWARE)
    {
        instance->work_mode = IIC_BLOCK_MODE;
        instance->soft_config = conf->soft_config;
        if (instance->soft_config.delay_us == 0U)
            instance->soft_config.delay_us = 2U;
        IICSoftInitGPIO(&instance->soft_config);
    }

    iic_instance[idx++] = instance;
    return instance;
}

void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode)
{ // HAL自带重入保护,不需要手动终止或等待传输完成
    if (iic == NULL)
        return;

    if (iic->bus_mode == IIC_BUS_SOFTWARE)
    {
        iic->work_mode = IIC_BLOCK_MODE;
        return;
    }

    if (iic->work_mode != mode)
    {
        iic->work_mode = mode; // 如果不同才需要修改
    }
}

uint8_t IICTransmit(IICInstance *iic, const uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    if ((iic == NULL) || (data == NULL) || (size == 0U))
        return 0U;

    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON)
        while (1)
            ; // 未知传输模式, 程序停止

    // 根据不同的工作模式进行不同的传输
    if (iic->bus_mode == IIC_BUS_SOFTWARE)
    {
        if (seq_mode != IIC_SEQ_RELEASE)
            while (1)
                ; // 软件IIC仅支持阻塞释放总线的传输方式
        return IICSoftTransmit(iic, data, size);
    }

    (void)seq_mode;
    return IICHardwareTransmit(iic, data, size);
}

void IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    if ((iic == NULL) || (data == NULL) || (size == 0U))
        return;

    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON)
        while (1)
            ; // 未知传输模式, 程序停止,请检查指针越界

    // 初始化接收缓冲区地址以及接受长度, 用于中断回调函数
    if (iic->bus_mode == IIC_BUS_SOFTWARE)
    {
        if (seq_mode != IIC_SEQ_RELEASE)
            while (1)
                ; // 软件IIC仅支持阻塞释放总线的传输方式
        (void)IICSoftReceive(iic, data, size);
        return;
    }

    iic->rx_buffer = data;
    iic->rx_len = size;

    (void)seq_mode;
    if (IICHardwareReceive(iic, data, size) && (iic->callback != NULL))
        iic->callback(iic);
}

void IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag)
{
    if ((iic == NULL) || (data == NULL) || (size == 0U))
        return;

    if (iic->bus_mode == IIC_BUS_SOFTWARE)
    {
        (void)IICSoftAccessMem(iic, mem_addr, data, size, mem_mode, mem8bit_flag);
        return;
    }

    {
        uint8_t address[2];
        uint8_t address_size = mem8bit_flag ? 1U : 2U;

        address[0] = (uint8_t)(mem_addr >> 8U);
        address[1] = (uint8_t)mem_addr;
        if (mem8bit_flag)
            address[0] = address[1];

        if (!IICHardwareTransmit(iic, address, address_size))
            return;

        if (mem_mode == IIC_WRITE_MEM)
            (void)IICHardwareTransmit(iic, data, size);
        else if (mem_mode == IIC_READ_MEM)
        {
            if (IICHardwareReceive(iic, data, size) && (iic->callback != NULL))
                iic->callback(iic);
        }
    }
}

static uint8_t IICConfigIsValid(const IIC_Init_Config_s *conf)
{
    if (conf == NULL)
        return 0U;

    if (conf->bus_mode == IIC_BUS_SOFTWARE)
        return IICSoftConfigIsValid(&conf->soft_config);

    if ((conf->bus_mode != IIC_BUS_HARDWARE) || (conf->handle == NULL) ||
        (conf->handle->Instance == NULL))
        return 0U;

    return 1U;
}

static uint8_t IICSoftConfigIsValid(const IIC_Soft_Config_s *conf)
{
    if (conf == NULL)
        return 0U;

    return ((conf->scl.GPIOx != NULL) &&
            (conf->sda.GPIOx != NULL) &&
            (conf->scl.GPIO_Pin != 0U) &&
            (conf->sda.GPIO_Pin != 0U));
}

static uint8_t IICInstanceMatches(const IICInstance *iic, const IIC_Init_Config_s *conf)
{
    if ((iic == NULL) || (conf == NULL))
        return 0U;

    if ((iic->bus_mode != conf->bus_mode) ||
        (iic->dev_address != conf->dev_address))
    {
        return 0U;
    }

    if (iic->bus_mode == IIC_BUS_HARDWARE)
        return (iic->handle == conf->handle);

    return ((iic->soft_config.scl.GPIOx == conf->soft_config.scl.GPIOx) &&
            (iic->soft_config.scl.GPIO_Pin == conf->soft_config.scl.GPIO_Pin) &&
            (iic->soft_config.sda.GPIOx == conf->soft_config.sda.GPIOx) &&
            (iic->soft_config.sda.GPIO_Pin == conf->soft_config.sda.GPIO_Pin));
}

static uint8_t IICHardwareTransmit(IICInstance *iic, const uint8_t *data, uint16_t size)
{
    I2C_Regs *i2c;
    uint16_t sent;

    if ((iic == NULL) || (iic->handle == NULL) ||
        (iic->handle->Instance == NULL) || (data == NULL) || (size == 0U))
    {
        return 0U;
    }

    i2c = iic->handle->Instance;
    if (!IICHardwareWaitComplete(i2c, 100U))
        return 0U;

    DL_I2C_flushControllerTXFIFO(i2c);
    sent = DL_I2C_fillControllerTXFIFO(i2c, data, size);
    DL_I2C_startControllerTransfer(i2c,
                                   iic->dev_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   size);

    while (sent < size)
    {
        uint16_t filled = DL_I2C_fillControllerTXFIFO(i2c, &data[sent], size - sent);
        if (filled > 0U)
            sent += filled;
        if (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR)
            return 0U;
    }

    return IICHardwareWaitComplete(i2c, 100U);
}

static uint8_t IICHardwareReceive(IICInstance *iic, uint8_t *data, uint16_t size)
{
    I2C_Regs *i2c;

    if ((iic == NULL) || (iic->handle == NULL) ||
        (iic->handle->Instance == NULL) || (data == NULL) || (size == 0U))
    {
        return 0U;
    }

    i2c = iic->handle->Instance;
    if (!IICHardwareWaitComplete(i2c, 100U))
        return 0U;

    DL_I2C_flushControllerRXFIFO(i2c);
    DL_I2C_startControllerTransfer(i2c,
                                   iic->dev_address,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   size);

    for (uint16_t i = 0U; i < size; i++)
    {
        uint32_t start_tick = HAL_GetTick();
        while (DL_I2C_isControllerRXFIFOEmpty(i2c))
        {
            if ((DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) ||
                ((HAL_GetTick() - start_tick) >= 100U))
            {
                return 0U;
            }
        }
        data[i] = DL_I2C_receiveControllerData(i2c);
    }

    return IICHardwareWaitComplete(i2c, 100U);
}

static uint8_t IICHardwareWaitComplete(I2C_Regs *i2c, uint32_t timeout_ms)
{
    uint32_t start_tick;

    if (i2c == NULL)
        return 0U;

    start_tick = HAL_GetTick();
    while (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        if ((DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) ||
            ((HAL_GetTick() - start_tick) >= timeout_ms))
        {
            return 0U;
        }
    }

    return ((DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U) ? 1U : 0U;
}

static void IICSoftInitGPIO(const IIC_Soft_Config_s *conf)
{
    if (!IICSoftConfigIsValid(conf))
        return;

    IICSoftInitGPIOPin(&conf->scl);
    IICSoftInitGPIOPin(&conf->sda);
    HAL_GPIO_WritePin(conf->scl.GPIOx, conf->scl.GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(conf->sda.GPIOx, conf->sda.GPIO_Pin, GPIO_PIN_SET);
}

static void IICSoftInitGPIOPin(const IIC_GPIO_Config_s *gpio)
{
    GPIO_InitTypeDef gpio_init;

    if ((gpio == NULL) || (gpio->GPIOx == NULL) || (gpio->GPIO_Pin == 0U))
        return;

    IICSoftEnableGPIOClock(gpio->GPIOx);
    gpio_init.Pin = gpio->GPIO_Pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(gpio->GPIOx, &gpio_init);

    /* 数字输出初始化默认关闭输入通道，软件 I2C 必须读回 SDA/SCL。 */
    IOMUX->SECCFG.PINCM[gpio->GPIO_IOMUX] |= IOMUX_PINCM_INENA_ENABLE;
}

static void IICSoftEnableGPIOClock(GPIO_TypeDef *GPIOx)
{
    /* MSPM0 的 GPIO 供电与引脚复用由 SysConfig 生成的初始化代码负责。 */
    (void)GPIOx;
}

static void IICSoftDelay(const IICInstance *iic)
{
    uint32_t delay_us = 2U;

    if ((iic != NULL) && (iic->soft_config.delay_us != 0U))
        delay_us = iic->soft_config.delay_us;

    BSP_DelayUs(delay_us);
}

static void IICSoftWriteSCL(const IICInstance *iic, GPIO_PinState state)
{
    HAL_GPIO_WritePin(iic->soft_config.scl.GPIOx, iic->soft_config.scl.GPIO_Pin, state);
}

static void IICSoftWriteSDA(const IICInstance *iic, GPIO_PinState state)
{
    HAL_GPIO_WritePin(iic->soft_config.sda.GPIOx, iic->soft_config.sda.GPIO_Pin, state);
}

static GPIO_PinState IICSoftReadSCL(const IICInstance *iic)
{
    return HAL_GPIO_ReadPin(iic->soft_config.scl.GPIOx, iic->soft_config.scl.GPIO_Pin);
}

static GPIO_PinState IICSoftReadSDA(const IICInstance *iic)
{
    return HAL_GPIO_ReadPin(iic->soft_config.sda.GPIOx, iic->soft_config.sda.GPIO_Pin);
}

static uint8_t IICSoftBusIsIdle(const IICInstance *iic)
{
    IICSoftWriteSCL(iic, GPIO_PIN_SET);
    IICSoftWriteSDA(iic, GPIO_PIN_SET);
    IICSoftDelay(iic);

    return ((IICSoftReadSCL(iic) == GPIO_PIN_SET) &&
            (IICSoftReadSDA(iic) == GPIO_PIN_SET)) ? 1U : 0U;
}

static void IICSoftStart(const IICInstance *iic)
{
    IICSoftWriteSDA(iic, GPIO_PIN_SET);
    IICSoftWriteSCL(iic, GPIO_PIN_SET);
    IICSoftDelay(iic);
    IICSoftWriteSDA(iic, GPIO_PIN_RESET);
    IICSoftDelay(iic);
    IICSoftWriteSCL(iic, GPIO_PIN_RESET);
}

static void IICSoftStop(const IICInstance *iic)
{
    IICSoftWriteSDA(iic, GPIO_PIN_RESET);
    IICSoftDelay(iic);
    IICSoftWriteSCL(iic, GPIO_PIN_SET);
    IICSoftDelay(iic);
    IICSoftWriteSDA(iic, GPIO_PIN_SET);
    IICSoftDelay(iic);
}

static void IICSoftWriteBit(const IICInstance *iic, uint8_t bit)
{
    IICSoftWriteSDA(iic, bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
    IICSoftDelay(iic);
    IICSoftWriteSCL(iic, GPIO_PIN_SET);
    IICSoftDelay(iic);
    IICSoftWriteSCL(iic, GPIO_PIN_RESET);
}

static uint8_t IICSoftReadBit(const IICInstance *iic)
{
    uint8_t bit;

    IICSoftWriteSDA(iic, GPIO_PIN_SET);
    IICSoftDelay(iic);
    IICSoftWriteSCL(iic, GPIO_PIN_SET);
    IICSoftDelay(iic);
    bit = (IICSoftReadSDA(iic) == GPIO_PIN_SET) ? 1U : 0U;
    IICSoftWriteSCL(iic, GPIO_PIN_RESET);

    return bit;
}

static uint8_t IICSoftWriteByte(const IICInstance *iic, uint8_t data)
{
    for (uint8_t i = 0; i < 8U; i++)
    {
        IICSoftWriteBit(iic, (uint8_t)(data & 0x80U));
        data <<= 1;
    }

    return (IICSoftReadBit(iic) == 0U) ? 1U : 0U;
}

static uint8_t IICSoftReadByte(const IICInstance *iic, uint8_t ack)
{
    uint8_t data = 0U;

    for (uint8_t i = 0; i < 8U; i++)
    {
        data <<= 1;
        data |= IICSoftReadBit(iic);
    }

    IICSoftWriteBit(iic, ack ? 0U : 1U);

    return data;
}

static uint8_t IICSoftTransmit(IICInstance *iic, const uint8_t *data, uint16_t size)
{
    if ((iic == NULL) || (data == NULL) || (size == 0U))
        return 0U;

    if (!IICSoftBusIsIdle(iic))
        return 0U;

    IICSoftStart(iic);
    if (!IICSoftWriteByte(iic, (uint8_t)(iic->dev_address << 1U)))
    {
        IICSoftStop(iic);
        return 0U;
    }

    for (uint16_t i = 0; i < size; i++)
    {
        if (!IICSoftWriteByte(iic, data[i]))
        {
            IICSoftStop(iic);
            return 0U;
        }
    }

    IICSoftStop(iic);
    return 1U;
}

static uint8_t IICSoftReceive(IICInstance *iic, uint8_t *data, uint16_t size)
{
    if ((iic == NULL) || (data == NULL) || (size == 0U))
        return 0U;

    IICSoftStart(iic);
    if (!IICSoftWriteByte(iic, (uint8_t)((iic->dev_address << 1U) | 1U)))
    {
        IICSoftStop(iic);
        return 0U;
    }

    for (uint16_t i = 0; i < size; i++)
        data[i] = IICSoftReadByte(iic, (i + 1U) < size);

    IICSoftStop(iic);
    return 1U;
}

static uint8_t IICSoftAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag)
{
    if ((iic == NULL) || (data == NULL) || (size == 0U))
        return 0U;

    IICSoftStart(iic);
    if (!IICSoftWriteByte(iic, (uint8_t)(iic->dev_address << 1U)))
    {
        IICSoftStop(iic);
        return 0U;
    }

    if (!mem8bit_flag && !IICSoftWriteByte(iic, (uint8_t)(mem_addr >> 8)))
    {
        IICSoftStop(iic);
        return 0U;
    }

    if (!IICSoftWriteByte(iic, (uint8_t)mem_addr))
    {
        IICSoftStop(iic);
        return 0U;
    }

    if (mem_mode == IIC_WRITE_MEM)
    {
        for (uint16_t i = 0; i < size; i++)
        {
            if (!IICSoftWriteByte(iic, data[i]))
            {
                IICSoftStop(iic);
                return 0U;
            }
        }
        IICSoftStop(iic);
        return 1U;
    }

    if (mem_mode == IIC_READ_MEM)
    {
        IICSoftStart(iic);
        if (!IICSoftWriteByte(iic, (uint8_t)((iic->dev_address << 1U) | 1U)))
        {
            IICSoftStop(iic);
            return 0U;
        }

        for (uint16_t i = 0; i < size; i++)
            data[i] = IICSoftReadByte(iic, (i + 1U) < size);

        IICSoftStop(iic);
        return 1U;
    }

    IICSoftStop(iic);
    while (1)
        ; // 未知模式, 程序停止
}
