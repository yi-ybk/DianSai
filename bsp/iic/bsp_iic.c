#include "bsp_iic.h"
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
static GPIO_PinState IICSoftReadSDA(const IICInstance *iic);
static void IICSoftStart(const IICInstance *iic);
static void IICSoftStop(const IICInstance *iic);
static void IICSoftWriteBit(const IICInstance *iic, uint8_t bit);
static uint8_t IICSoftReadBit(const IICInstance *iic);
static uint8_t IICSoftWriteByte(const IICInstance *iic, uint8_t data);
static uint8_t IICSoftReadByte(const IICInstance *iic, uint8_t ack);
static uint8_t IICSoftTransmit(IICInstance *iic, const uint8_t *data, uint16_t size);
static uint8_t IICSoftReceive(IICInstance *iic, uint8_t *data, uint16_t size);
static uint8_t IICSoftAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag);

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

    instance->dev_address = conf->dev_address << 1; // 地址左移一位,最低位为读写位
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

void IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    if ((iic == NULL) || (data == NULL) || (size == 0U))
        return;

    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON)
        while (1)
            ; // 未知传输模式, 程序停止

    // 根据不同的工作模式进行不同的传输
    if (iic->bus_mode == IIC_BUS_SOFTWARE)
    {
        if (seq_mode != IIC_SEQ_RELEASE)
            while (1)
                ; // 软件IIC仅支持阻塞释放总线的传输方式
        (void)IICSoftTransmit(iic, data, size);
        return;
    }

#ifdef HAL_I2C_MODULE_ENABLED
    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        if (seq_mode != IIC_SEQ_RELEASE)
            while (1)
                ;                                                                // 阻塞模式下不支持HOLD ON模式!!!只能传输完成后立刻释放总线
        HAL_I2C_Master_Transmit(iic->handle, iic->dev_address, data, size, 100); // 默认超时时间100ms
        break;
    case IIC_IT_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    case IIC_DMA_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    default:
        while (1)
            ; // 未知传输模式, 程序停止
    }
#endif
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

#ifdef HAL_I2C_MODULE_ENABLED
    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        if (seq_mode != IIC_SEQ_RELEASE)
            while (1)
                ;                                                               // 阻塞模式下不支持HOLD ON模式!!!
        HAL_I2C_Master_Receive(iic->handle, iic->dev_address, data, size, 100); // 默认超时时间100ms
        break;
    case IIC_IT_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    case IIC_DMA_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    default:
        while (1)
            ; // 未知传输模式, 程序停止
    }
#endif
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

#ifdef HAL_I2C_MODULE_ENABLED
    uint16_t bit_flag = mem8bit_flag ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;

    if (mem_mode == IIC_WRITE_MEM)
    {
        HAL_I2C_Mem_Write(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size, 100);
    }
    else if (mem_mode == IIC_READ_MEM)
    {
        HAL_I2C_Mem_Read(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size, 100);
    }
    else
    {
        while (1)
            ; // 未知模式, 程序停止
    }
#endif
}

#ifdef HAL_I2C_MODULE_ENABLED
/**
 * @brief IIC接收完成回调函数
 *
 * @param hi2c handle
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 如果是当前i2c硬件发出的complete,且dev_address和之前发起接收的地址相同,同时回到函数不为空, 则调用回调函数
    for (uint8_t i = 0; i < idx; i++)
    {
        if ((iic_instance[i] != NULL) &&
            (iic_instance[i]->bus_mode == IIC_BUS_HARDWARE) &&
            (iic_instance[i]->handle == hi2c) &&
            (hi2c->Devaddress == iic_instance[i]->dev_address))
        {
            if (iic_instance[i]->callback != NULL) // 回调函数不为空
                iic_instance[i]->callback(iic_instance[i]);
            return;
        }
    }
}

/**
 * @brief 内存访问回调函数,仅做形式上的封装,仍然使用HAL_I2C_MasterRxCpltCallback
 *
 * @param hi2c handle
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_MasterRxCpltCallback(hi2c);
}
#endif

static uint8_t IICConfigIsValid(const IIC_Init_Config_s *conf)
{
    if (conf == NULL)
        return 0U;

    if (conf->bus_mode == IIC_BUS_SOFTWARE)
        return IICSoftConfigIsValid(&conf->soft_config);

#ifdef HAL_I2C_MODULE_ENABLED
    if ((conf->bus_mode != IIC_BUS_HARDWARE) || (conf->handle == NULL))
        return 0U;

    return 1U;
#else
    return 0U;
#endif
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
        (iic->dev_address != (uint8_t)(conf->dev_address << 1)))
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
}

static void IICSoftEnableGPIOClock(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (GPIOx == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (GPIOx == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (GPIOx == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (GPIOx == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
#ifdef GPIOF
    else if (GPIOx == GPIOF)
        __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
    else if (GPIOx == GPIOG)
        __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
#ifdef GPIOH
    else if (GPIOx == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
#ifdef GPIOI
    else if (GPIOx == GPIOI)
        __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
}

static void IICSoftDelay(const IICInstance *iic)
{
    uint32_t delay_us = 2U;
    volatile uint32_t delay_loop;

    if ((iic != NULL) && (iic->soft_config.delay_us != 0U))
        delay_us = iic->soft_config.delay_us;

    delay_loop = delay_us * 64U;
    while (delay_loop-- > 0U)
        __NOP();
}

static void IICSoftWriteSCL(const IICInstance *iic, GPIO_PinState state)
{
    HAL_GPIO_WritePin(iic->soft_config.scl.GPIOx, iic->soft_config.scl.GPIO_Pin, state);
}

static void IICSoftWriteSDA(const IICInstance *iic, GPIO_PinState state)
{
    HAL_GPIO_WritePin(iic->soft_config.sda.GPIOx, iic->soft_config.sda.GPIO_Pin, state);
}

static GPIO_PinState IICSoftReadSDA(const IICInstance *iic)
{
    return HAL_GPIO_ReadPin(iic->soft_config.sda.GPIOx, iic->soft_config.sda.GPIO_Pin);
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

    IICSoftStart(iic);
    if (!IICSoftWriteByte(iic, (uint8_t)(iic->dev_address | 0U)))
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
    if (!IICSoftWriteByte(iic, (uint8_t)(iic->dev_address | 1U)))
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
    if (!IICSoftWriteByte(iic, (uint8_t)(iic->dev_address | 0U)))
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
        if (!IICSoftWriteByte(iic, (uint8_t)(iic->dev_address | 1U)))
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
