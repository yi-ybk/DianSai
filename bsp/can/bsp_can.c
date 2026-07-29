/**
 * @file    bsp_can.c
 * @brief   CAN设备底层驱动实现
 * @details 保留原工程的逻辑实例使用方式，并以MSPM0 MCAN DriverLib完成底层适配。
 */
#include "bsp_can.h"
#include "bsp_delay.h"
#include <string.h>

typedef struct
{
    uint32_t id;
    uint8_t length;
    CANFrameType_t frame_type;
    uint8_t data[CAN_CLASSIC_DATA_MAX_LEN];
} CANRxFrame_t;

/* 使用静态实例池，避免在嵌入式运行过程中动态分配内存。 */
static uint8_t idx;
static CANInstance can_instance_pool[CAN_MX_REGISTER_CNT];
static CANInstance *can_instance[CAN_MX_REGISTER_CNT];

static bool CANIdIsValid(uint32_t id, CANFrameType_t frame_type);
static bool CANReceive(CANInstance *can, CANRxFrame_t *frame);
static bool CANFrameMatches(const CANInstance *can, const CANRxFrame_t *frame);

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *instance;
    uint8_t instance_index;

    if ((config == NULL) || (config->mcan == NULL) ||
        (config->tx_buffer_index >= 32U) ||
        ((config->rx_fifo_number != DL_MCAN_RX_FIFO_NUM_0) &&
         (config->rx_fifo_number != DL_MCAN_RX_FIFO_NUM_1)) ||
        !CANIdIsValid(config->tx_id, config->tx_frame_type) ||
        !CANIdIsValid(config->rx_id, config->rx_frame_type))
    {
        return NULL;
    }

    for (instance_index = 0U; instance_index < idx; ++instance_index)
    {
        if ((can_instance[instance_index]->mcan == config->mcan) &&
            (can_instance[instance_index]->rx_id == config->rx_id) &&
            (can_instance[instance_index]->rx_frame_type == config->rx_frame_type))
        {
            return NULL;
        }
    }

    if (idx >= CAN_MX_REGISTER_CNT)
        return NULL;

    instance = &can_instance_pool[idx];
    memset(instance, 0, sizeof(*instance));

    instance->mcan            = config->mcan;
    instance->irq_number      = config->irq_number;
    instance->tx_buffer_index = config->tx_buffer_index;
    instance->rx_fifo_number  = config->rx_fifo_number;
    instance->tx_id           = config->tx_id;
    instance->tx_len          = CAN_CLASSIC_DATA_MAX_LEN;
    instance->rx_id           = config->rx_id;
    instance->rx_id_mask      = config->rx_id_mask;
    instance->tx_frame_type   = config->tx_frame_type;
    instance->rx_frame_type   = config->rx_frame_type;
    instance->module_callback = config->module_callback;
    instance->id              = config->id;

    if (instance->rx_id_mask == 0U)
    {
        instance->rx_id_mask = instance->rx_frame_type == CAN_FRAME_EXTENDED ?
                               0x1FFFFFFFU : 0x7FFU;
    }

    can_instance[idx++] = instance;
    NVIC_ClearPendingIRQ(instance->irq_number);
    NVIC_EnableIRQ(instance->irq_number);
    return instance;
}

void CANSetDLC(CANInstance *can, uint8_t length)
{
    if ((can != NULL) && (length <= CAN_CLASSIC_DATA_MAX_LEN))
        can->tx_len = length;
}

void CANSetTxId(CANInstance *can, uint32_t tx_id)
{
    if ((can != NULL) && CANIdIsValid(tx_id, can->tx_frame_type))
        can->tx_id = tx_id;
}

bool CANTransmit(CANInstance *can, float timeout_ms)
{
    DL_MCAN_TxBufElement element;

    if ((can == NULL) || (can->mcan == NULL) ||
        (can->tx_len > CAN_CLASSIC_DATA_MAX_LEN) ||
        !CANIdIsValid(can->tx_id, can->tx_frame_type) ||
        !CANWaitForTxComplete(can, timeout_ms))
    {
        return false;
    }

    memset(&element, 0, sizeof(element));
    element.id = can->tx_frame_type == CAN_FRAME_EXTENDED ?
                 can->tx_id : (can->tx_id << 18U);
    element.xtd = can->tx_frame_type == CAN_FRAME_EXTENDED ? 1U : 0U;
    element.dlc = can->tx_len;
    element.fdf = 0U;
    element.brs = 0U;
    memcpy(element.data, can->tx_buff, can->tx_len);

    DL_MCAN_writeMsgRam(can->mcan,
                        DL_MCAN_MEM_TYPE_BUF,
                        can->tx_buffer_index,
                        &element);
    return DL_MCAN_TXBufAddReq(can->mcan, can->tx_buffer_index) == 0;
}

bool CANWaitForTxComplete(CANInstance *can, float timeout_ms)
{
    uint32_t elapsed_us = 0U;
    uint32_t timeout_us;

    if ((can == NULL) || (can->mcan == NULL) ||
        (timeout_ms < 0.0f) || (timeout_ms != timeout_ms))
    {
        return false;
    }

    timeout_us = (uint32_t)(timeout_ms * 1000.0f + 0.5f);
    do
    {
        if ((DL_MCAN_getTxBufReqPend(can->mcan) &
             (1UL << can->tx_buffer_index)) == 0U)
        {
            return true;
        }

        if (elapsed_us >= timeout_us)
            return false;

        BSP_DelayUs(10U);
        elapsed_us += 10U;
    } while (true);
}

void CANService(CANInstance *can)
{
    CANRxFrame_t frame;
    uint8_t instance_index;

    if (can == NULL)
        return;

    while (CANReceive(can, &frame))
    {
        for (instance_index = 0U; instance_index < idx; ++instance_index)
        {
            CANInstance *receiver = can_instance[instance_index];

            if ((receiver->mcan != can->mcan) ||
                (receiver->rx_fifo_number != can->rx_fifo_number) ||
                !CANFrameMatches(receiver, &frame))
            {
                continue;
            }

            receiver->rx_message_id = frame.id;
            receiver->rx_len = frame.length;
            memcpy(receiver->rx_buff, frame.data, frame.length);
            if (receiver->module_callback != NULL)
                receiver->module_callback(receiver);

            /* 与原工程一致：一帧只交给第一个匹配的逻辑实例。 */
            break;
        }
    }
}

void CANIRQHandler(MCAN_Regs *mcan, uint32_t rx_fifo_number)
{
    CANInstance *service_instance = NULL;
    CANInstance discard_instance = {
        .mcan = mcan,
        .rx_fifo_number = rx_fifo_number,
    };
    CANRxFrame_t discarded_frame;
    uint32_t interrupt_status;
    uint8_t instance_index;

    if ((mcan == NULL) ||
        ((rx_fifo_number != DL_MCAN_RX_FIFO_NUM_0) &&
         (rx_fifo_number != DL_MCAN_RX_FIFO_NUM_1)))
    {
        return;
    }

    if (DL_MCAN_getPendingInterrupt(mcan) != DL_MCAN_IIDX_LINE1)
        return;

    interrupt_status = DL_MCAN_getIntrStatus(mcan);
    DL_MCAN_clearIntrStatus(
        mcan, interrupt_status, DL_MCAN_INTR_SRC_MCAN_LINE_1);

    for (instance_index = 0U; instance_index < idx; ++instance_index)
    {
        if ((can_instance[instance_index]->mcan == mcan) &&
            (can_instance[instance_index]->rx_fifo_number == rx_fifo_number))
        {
            service_instance = can_instance[instance_index];
            break;
        }
    }

    if (service_instance != NULL)
    {
        CANService(service_instance);
        return;
    }

    while (CANReceive(&discard_instance, &discarded_frame))
    {
    }
}

static bool CANReceive(CANInstance *can, CANRxFrame_t *frame)
{
    DL_MCAN_RxBufElement element;
    DL_MCAN_RxFIFOStatus fifo_status;

    if ((can == NULL) || (can->mcan == NULL) || (frame == NULL))
        return false;

    memset(&fifo_status, 0, sizeof(fifo_status));
    fifo_status.num = can->rx_fifo_number;
    DL_MCAN_getRxFIFOStatus(can->mcan, &fifo_status);
    if (fifo_status.fillLvl == 0U)
        return false;

    memset(&element, 0, sizeof(element));
    DL_MCAN_readMsgRam(can->mcan,
                       DL_MCAN_MEM_TYPE_FIFO,
                       0U,
                       can->rx_fifo_number,
                       &element);
    if (DL_MCAN_writeRxFIFOAck(can->mcan,
                               can->rx_fifo_number,
                               fifo_status.getIdx) != 0)
    {
        return false;
    }

    frame->frame_type = element.xtd != 0U ?
                        CAN_FRAME_EXTENDED : CAN_FRAME_STANDARD;
    frame->id = frame->frame_type == CAN_FRAME_EXTENDED ?
                element.id : ((element.id >> 18U) & 0x7FFU);
    frame->length = element.dlc <= CAN_CLASSIC_DATA_MAX_LEN ?
                    (uint8_t)element.dlc : CAN_CLASSIC_DATA_MAX_LEN;
    memcpy(frame->data, element.data, frame->length);
    return true;
}

static bool CANFrameMatches(const CANInstance *can, const CANRxFrame_t *frame)
{
    if ((can == NULL) || (frame == NULL) ||
        (can->rx_frame_type != frame->frame_type))
    {
        return false;
    }

    return ((frame->id ^ can->rx_id) & can->rx_id_mask) == 0U;
}

static bool CANIdIsValid(uint32_t id, CANFrameType_t frame_type)
{
    if (frame_type == CAN_FRAME_STANDARD)
        return id <= 0x7FFU;
    if (frame_type == CAN_FRAME_EXTENDED)
        return id <= 0x1FFFFFFFU;
    return false;
}
