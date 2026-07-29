/**
 * @file    zhangdatou_42.c
 * @brief   张大头ZDT_X42S闭环步进电机驱动实现
 * @details 完成CAN扩展帧分包发送、常用运动控制和实时反馈解析。
 */
#include "zhangdatou_42.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define ZDT42_CAN_ID(_motor_id, _packet) \
    ((((uint32_t)(_motor_id)) << 8U) | (uint32_t)(_packet))

static void Zdt42BindMethods(Zdt42_t *motor);
static void Zdt42CanCallback(CANInstance *instance);
static uint32_t Zdt42PulseMagnitude(int32_t pulse);
static uint16_t Zdt42LimitSpeed(uint16_t speed_rpm);
static uint32_t Zdt42GetTickMsFromISR(void);
static bool Zdt42SendToSelf(Zdt42_t *motor, const uint8_t *command, uint16_t length);

/**
 * @brief   初始化张大头42步进电机对象
 * @param   motor 电机对象指针
 * @param   config 初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool Zdt42Init(Zdt42_t *motor, const Zdt42InitConfig_t *config)
{
    CAN_Init_Config_s can_config = {0};

    if ((motor == NULL) || (config == NULL) ||
        (config->mcan == NULL) || (config->motor_id == 0U))
        return false;

    Zdt42BindMethods(motor);
    if (motor->initialized)
        return true;

    motor->init_config = *config;
    if (motor->init_config.checksum == 0U)
        motor->init_config.checksum = ZDT42_DEFAULT_CHECKSUM;
    if (motor->init_config.tx_timeout_ms <= 0.0f)
        motor->init_config.tx_timeout_ms = ZDT42_DEFAULT_TX_TIMEOUT_MS;

    memset(&motor->data, 0, sizeof(motor->data));
    motor->feedback_callback = NULL;
    motor->feedback_context = NULL;

    can_config.mcan = motor->init_config.mcan;
    can_config.irq_number = motor->init_config.irq_number;
    can_config.tx_buffer_index = motor->init_config.tx_buffer_index;
    can_config.rx_fifo_number = motor->init_config.rx_fifo_number;
    can_config.tx_id = ZDT42_CAN_ID(motor->init_config.motor_id, 0U);
    can_config.rx_id = ZDT42_CAN_ID(motor->init_config.motor_id, 0U);
    can_config.rx_id_mask = 0x1FFFFF00U;
    can_config.tx_frame_type = CAN_FRAME_EXTENDED;
    can_config.rx_frame_type = CAN_FRAME_EXTENDED;
    can_config.module_callback = Zdt42CanCallback;
    can_config.id = motor;

    motor->can = CANRegister(&can_config);
    if (motor->can == NULL)
        return false;

    motor->initialized = true;
    return true;
}

/** @brief 设置电机使能状态 */
bool Zdt42Enable(Zdt42_t *motor, bool enabled, bool sync)
{
    uint8_t command[5];

    if ((motor == NULL) || (!motor->initialized))
        return false;

    command[0] = 0xF3U;
    command[1] = 0xABU;
    command[2] = enabled ? 1U : 0U;
    command[3] = sync ? 1U : 0U;
    command[4] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/**
 * @brief   设置速度模式目标转速
 * @details 正值为CW，负值为CCW；绝对值限制为0~5000RPM。
 */
bool Zdt42SetSpeed(Zdt42_t *motor, float speed_rpm, uint8_t acceleration, bool sync)
{
    uint16_t speed;
    uint8_t command[7];

    if ((motor == NULL) || (!motor->initialized) || (speed_rpm != speed_rpm))
        return false;

    if (speed_rpm > ZDT42_MAX_SPEED_RPM)
        speed_rpm = ZDT42_MAX_SPEED_RPM;
    else if (speed_rpm < -ZDT42_MAX_SPEED_RPM)
        speed_rpm = -ZDT42_MAX_SPEED_RPM;

    speed = (uint16_t)(speed_rpm < 0.0f ? -speed_rpm : speed_rpm);
    command[0] = 0xF6U;
    command[1] = speed_rpm < 0.0f ? 1U : 0U;
    command[2] = (uint8_t)(speed >> 8U);
    command[3] = (uint8_t)speed;
    command[4] = acceleration;
    command[5] = sync ? 1U : 0U;
    command[6] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 使用梯形位置模式运动 */
bool Zdt42MovePosition(Zdt42_t *motor, int32_t pulse, uint16_t speed_rpm,
                       uint8_t acceleration, Zdt42PositionMode_t mode, bool sync)
{
    uint32_t magnitude;
    uint8_t command[12];

    if ((motor == NULL) || (!motor->initialized) || (mode > ZDT42_POSITION_RELATIVE_CURRENT))
        return false;

    speed_rpm = Zdt42LimitSpeed(speed_rpm);
    magnitude = Zdt42PulseMagnitude(pulse);
    command[0] = 0xFDU;
    command[1] = pulse < 0 ? 1U : 0U;
    command[2] = (uint8_t)(speed_rpm >> 8U);
    command[3] = (uint8_t)speed_rpm;
    command[4] = acceleration;
    command[5] = (uint8_t)(magnitude >> 24U);
    command[6] = (uint8_t)(magnitude >> 16U);
    command[7] = (uint8_t)(magnitude >> 8U);
    command[8] = (uint8_t)magnitude;
    command[9] = (uint8_t)mode;
    command[10] = sync ? 1U : 0U;
    command[11] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 配置快速位置模式参数 */
bool Zdt42SetQuickPosition(Zdt42_t *motor, uint16_t speed_rpm,
                           uint8_t acceleration, Zdt42PositionMode_t mode, bool sync)
{
    uint8_t command[7];

    if ((motor == NULL) || (!motor->initialized) || (mode > ZDT42_POSITION_RELATIVE_CURRENT))
        return false;

    speed_rpm = Zdt42LimitSpeed(speed_rpm);
    command[0] = 0xF1U;
    command[1] = (uint8_t)(speed_rpm >> 8U);
    command[2] = (uint8_t)speed_rpm;
    command[3] = acceleration;
    command[4] = (uint8_t)mode;
    command[5] = sync ? 1U : 0U;
    command[6] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 下发快速位置模式脉冲数 */
bool Zdt42MoveQuick(Zdt42_t *motor, int32_t pulse)
{
    uint32_t value = (uint32_t)pulse;
    uint8_t command[6];

    if ((motor == NULL) || (!motor->initialized))
        return false;

    command[0] = 0xFCU;
    command[1] = (uint8_t)(value >> 24U);
    command[2] = (uint8_t)(value >> 16U);
    command[3] = (uint8_t)(value >> 8U);
    command[4] = (uint8_t)value;
    command[5] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 立即停止电机 */
bool Zdt42Stop(Zdt42_t *motor, bool sync)
{
    uint8_t command[4];

    if ((motor == NULL) || (!motor->initialized))
        return false;
    command[0] = 0xFEU;
    command[1] = 0x98U;
    command[2] = sync ? 1U : 0U;
    command[3] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 触发电机回零 */
bool Zdt42Home(Zdt42_t *motor, Zdt42HomeMode_t mode, bool sync)
{
    uint8_t command[4];

    if ((motor == NULL) || (!motor->initialized) || (mode > ZDT42_HOME_LIMIT_SWITCH))
        return false;
    command[0] = 0x9AU;
    command[1] = (uint8_t)mode;
    command[2] = sync ? 1U : 0U;
    command[3] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 强制中断回零 */
bool Zdt42InterruptHome(Zdt42_t *motor)
{
    uint8_t command[3];

    if ((motor == NULL) || (!motor->initialized))
        return false;
    command[0] = 0x9CU;
    command[1] = 0x48U;
    command[2] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 将当前位置设为单圈回零零点 */
bool Zdt42SetOrigin(Zdt42_t *motor, bool save)
{
    uint8_t command[4];

    if ((motor == NULL) || (!motor->initialized))
        return false;
    command[0] = 0x93U;
    command[1] = 0x88U;
    command[2] = save ? 1U : 0U;
    command[3] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 将当前位置清零 */
bool Zdt42ResetPosition(Zdt42_t *motor)
{
    uint8_t command[3];

    if ((motor == NULL) || (!motor->initialized))
        return false;
    command[0] = 0x0AU;
    command[1] = 0x6DU;
    command[2] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 解除堵转、过热和过流保护 */
bool Zdt42ClearFault(Zdt42_t *motor)
{
    uint8_t command[3];

    if ((motor == NULL) || (!motor->initialized))
        return false;
    command[0] = 0x0EU;
    command[1] = 0x52U;
    command[2] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 广播触发已缓存的多机同步命令 */
bool Zdt42TriggerSync(Zdt42_t *motor)
{
    uint8_t command[3];

    if ((motor == NULL) || (!motor->initialized))
        return false;
    command[0] = 0xFFU;
    command[1] = 0x66U;
    command[2] = motor->init_config.checksum;
    return Zdt42SendRaw(motor, 0U, command, sizeof(command));
}

/** @brief 请求读取指定系统参数 */
bool Zdt42ReadParameter(Zdt42_t *motor, Zdt42SystemParameter_t parameter)
{
    uint8_t command[2];

    if ((motor == NULL) || (!motor->initialized))
        return false;
    command[0] = (uint8_t)parameter;
    command[1] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/** @brief 设置指定系统参数的定时返回周期，周期为0时停止返回 */
bool Zdt42SetAutoReturn(Zdt42_t *motor, Zdt42SystemParameter_t parameter, uint16_t period_ms)
{
    uint8_t command[6];

    if ((motor == NULL) || (!motor->initialized))
        return false;
    command[0] = 0x11U;
    command[1] = 0x18U;
    command[2] = (uint8_t)parameter;
    command[3] = (uint8_t)(period_ms >> 8U);
    command[4] = (uint8_t)period_ms;
    command[5] = motor->init_config.checksum;
    return Zdt42SendToSelf(motor, command, sizeof(command));
}

/**
 * @brief   发送厂家原始命令
 * @details command包含功能码、参数和校验字节，不包含电机地址；长命令自动分包。
 */
bool Zdt42SendRaw(Zdt42_t *motor, uint8_t motor_id,
                  const uint8_t *command, uint16_t length)
{
    uint16_t offset = 1U;
    uint8_t packet = 0U;

    if ((motor == NULL) || (!motor->initialized) ||
        (command == NULL) || (length < 2U))
        return false;

    taskENTER_CRITICAL();
    motor->data.command_received = false;
    motor->data.motion_complete = false;
    taskEXIT_CRITICAL();

    while (offset < length)
    {
        uint8_t payload_length = (uint8_t)(length - offset);
        uint8_t i;

        if (payload_length > 7U)
            payload_length = 7U;
        CANSetTxId(motor->can, ZDT42_CAN_ID(motor_id, packet));
        CANSetDLC(motor->can, (uint8_t)(payload_length + 1U));
        motor->can->tx_buff[0] = command[0];
        for (i = 0U; i < payload_length; i++)
            motor->can->tx_buff[i + 1U] = command[offset + i];

        if (!CANTransmit(motor->can, motor->init_config.tx_timeout_ms))
        {
            motor->data.tx_error_count++;
            return false;
        }

        motor->data.tx_frame_count++;
        offset += payload_length;
        packet++;
    }

    if (!CANWaitForTxComplete(motor->can,
                              motor->init_config.tx_timeout_ms * (float)(packet + 1U)))
    {
        motor->data.tx_error_count++;
        return false;
    }

    return true;
}

/** @brief 获取电机反馈的一致快照 */
void Zdt42GetData(Zdt42_t *motor, Zdt42Data_t *data)
{
    if ((motor == NULL) || (data == NULL))
        return;

    taskENTER_CRITICAL();
    memcpy(data, &motor->data, sizeof(*data));
    taskEXIT_CRITICAL();
}

/** @brief 设置反馈回调函数 */
void Zdt42SetCallback(Zdt42_t *motor, Zdt42FeedbackCallback_t callback, void *context)
{
    if (motor == NULL)
        return;

    taskENTER_CRITICAL();
    motor->feedback_callback = callback;
    motor->feedback_context = context;
    taskEXIT_CRITICAL();
}

/** @brief 解析CAN反馈，运行于CAN接收中断上下文 */
static void Zdt42CanCallback(CANInstance *instance)
{
    Zdt42_t *motor;
    const uint8_t *frame_data;
    uint8_t length;

    if ((instance == NULL) || (instance->id == NULL))
        return;
    motor = (Zdt42_t *)instance->id;
    frame_data = instance->rx_buff;
    length = instance->rx_len;
    if ((length < 2U) || (length > 8U))
        return;

    if (frame_data[length - 1U] != motor->init_config.checksum)
    {
        motor->data.checksum_error_count++;
        return;
    }

    memcpy(motor->data.raw_data, frame_data, length);
    motor->data.raw_length = length;
    motor->data.last_function = frame_data[0];
    motor->data.last_response = length > 2U ? frame_data[1] : 0U;
    motor->data.last_packet_index = (uint8_t)(instance->rx_message_id & 0xFFU);
    if (length == 3U)
    {
        motor->data.command_received = frame_data[1] == 0x02U;
        motor->data.motion_complete = frame_data[1] == 0x9FU;
    }

    if ((frame_data[0] == ZDT42_PARAM_SPEED) && (length == 5U))
    {
        uint16_t speed = ((uint16_t)frame_data[2] << 8U) |
                         (uint16_t)frame_data[3];
        motor->data.speed_rpm = frame_data[1] != 0U ? -(float)speed : (float)speed;
    }
    else if ((frame_data[0] == ZDT42_PARAM_POSITION) && (length == 7U))
    {
        uint32_t position = ((uint32_t)frame_data[2] << 24U) |
                            ((uint32_t)frame_data[3] << 16U) |
                            ((uint32_t)frame_data[4] << 8U) |
                            (uint32_t)frame_data[5];
        motor->data.position_deg = (float)position * 360.0f / ZDT42_POSITION_COUNTS_PER_REV;
        if (frame_data[1] != 0U)
            motor->data.position_deg = -motor->data.position_deg;
    }

    motor->data.rx_frame_count++;
    motor->data.last_rx_tick = Zdt42GetTickMsFromISR();
    if (motor->feedback_callback != NULL)
        motor->feedback_callback(motor, &motor->data, motor->feedback_context);
}

static bool Zdt42SendToSelf(Zdt42_t *motor, const uint8_t *command, uint16_t length)
{
    return Zdt42SendRaw(motor, motor->init_config.motor_id, command, length);
}

static uint32_t Zdt42PulseMagnitude(int32_t pulse)
{
    if (pulse >= 0)
        return (uint32_t)pulse;
    return (uint32_t)(-(pulse + 1)) + 1U;
}

static uint16_t Zdt42LimitSpeed(uint16_t speed_rpm)
{
    return speed_rpm > (uint16_t)ZDT42_MAX_SPEED_RPM ?
           (uint16_t)ZDT42_MAX_SPEED_RPM : speed_rpm;
}

static uint32_t Zdt42GetTickMsFromISR(void)
{
    return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
}

static void Zdt42BindMethods(Zdt42_t *motor)
{
    motor->init               = Zdt42Init;
    motor->enable             = Zdt42Enable;
    motor->set_speed          = Zdt42SetSpeed;
    motor->move_position      = Zdt42MovePosition;
    motor->set_quick_position = Zdt42SetQuickPosition;
    motor->move_quick         = Zdt42MoveQuick;
    motor->stop               = Zdt42Stop;
    motor->home               = Zdt42Home;
    motor->interrupt_home     = Zdt42InterruptHome;
    motor->set_origin         = Zdt42SetOrigin;
    motor->reset_position     = Zdt42ResetPosition;
    motor->clear_fault        = Zdt42ClearFault;
    motor->trigger_sync       = Zdt42TriggerSync;
    motor->read_parameter     = Zdt42ReadParameter;
    motor->set_auto_return    = Zdt42SetAutoReturn;
    motor->send_raw           = Zdt42SendRaw;
    motor->get_data           = Zdt42GetData;
    motor->set_callback       = Zdt42SetCallback;
}
