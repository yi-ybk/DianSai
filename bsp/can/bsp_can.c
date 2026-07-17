/**
 * @file    bsp_can.c
 * @brief   CAN-FD设备底层驱动实现
 * @details 使用MSPM0 MCAN DriverLib封装帧收发，通过实例池管理已注册外设。
 */
#include "bsp_can.h"
#include <string.h>

/* -------------------- 静态变量区 -------------------- */
/** @brief 已注册的CAN实例总数 */
static uint8_t idx;
/** @brief 预分配的全局CAN实例静态内存池 */
static CANInstance can_instance_pool[CAN_DEVICE_CNT];
/** @brief 全局注册的CAN实例指针数组 */
static CANInstance *can_instance[CAN_DEVICE_CNT];

static bool CANLengthToDLC(uint8_t length, uint32_t *dlc);
static uint8_t CANDLCToLength(uint32_t dlc);
static bool CANFrameToTxElement(const CANFrame *frame, DL_MCAN_TxBufElement *element);
static void CANRxElementToFrame(const DL_MCAN_RxBufElement *element, CANFrame *frame);

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *instance;

    if ((config == NULL) ||
        (config->mcan == NULL) ||
        (config->tx_buffer_index >= 32U) ||
        ((config->rx_fifo_number != DL_MCAN_RX_FIFO_NUM_0) &&
         (config->rx_fifo_number != DL_MCAN_RX_FIFO_NUM_1)))
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < idx; i++)
    {
        if (can_instance[i]->mcan == config->mcan)
            return can_instance[i];
    }

    if (idx >= CAN_DEVICE_CNT)
        return NULL;

    instance = &can_instance_pool[idx];
    memset(instance, 0, sizeof(*instance));

    instance->mcan            = config->mcan;
    instance->tx_buffer_index = config->tx_buffer_index;
    instance->rx_fifo_number  = config->rx_fifo_number;
    instance->module_callback = config->module_callback;
    instance->id              = config->id;

    can_instance[idx++] = instance;
    return instance;
}

bool CANTransmit(CANInstance *can, const CANFrame *frame)
{
    DL_MCAN_TxBufElement element;
    uint32_t pending_mask;

    if ((can == NULL) || (can->mcan == NULL) ||
        !CANFrameToTxElement(frame, &element))
    {
        return false;
    }

    pending_mask = DL_MCAN_getTxBufReqPend(can->mcan);
    if ((pending_mask & (1UL << can->tx_buffer_index)) != 0U)
        return false;

    DL_MCAN_writeMsgRam(can->mcan,
                         DL_MCAN_MEM_TYPE_BUF,
                         can->tx_buffer_index,
                         &element);
    return DL_MCAN_TXBufAddReq(can->mcan, can->tx_buffer_index) == 0;
}

bool CANReceive(CANInstance *can, CANFrame *frame)
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
    if (DL_MCAN_writeRxFIFOAck(can->mcan, can->rx_fifo_number, fifo_status.getIdx) != 0)
        return false;

    CANRxElementToFrame(&element, frame);
    return true;
}

void CANService(CANInstance *can)
{
    if ((can == NULL) || !CANReceive(can, &can->rx_frame))
        return;

    if (can->module_callback != NULL)
        can->module_callback(can);
}

static bool CANLengthToDLC(uint8_t length, uint32_t *dlc)
{
    if (dlc == NULL)
        return false;

    if (length <= 8U)
    {
        *dlc = length;
        return true;
    }

    switch (length)
    {
    case 12U: *dlc = 9U;  break;
    case 16U: *dlc = 10U; break;
    case 20U: *dlc = 11U; break;
    case 24U: *dlc = 12U; break;
    case 32U: *dlc = 13U; break;
    case 48U: *dlc = 14U; break;
    case 64U: *dlc = 15U; break;
    default: return false;
    }

    return true;
}

static uint8_t CANDLCToLength(uint32_t dlc)
{
    if (dlc <= 8U)
        return (uint8_t)dlc;

    switch (dlc)
    {
    case 9U:  return 12U;
    case 10U: return 16U;
    case 11U: return 20U;
    case 12U: return 24U;
    case 13U: return 32U;
    case 14U: return 48U;
    case 15U: return 64U;
    default:  return 0U;
    }
}

static bool CANFrameToTxElement(const CANFrame *frame, DL_MCAN_TxBufElement *element)
{
    uint32_t dlc;

    if ((frame == NULL) || (element == NULL) ||
        !CANLengthToDLC(frame->length, &dlc) ||
        ((!frame->is_fd) && (frame->length > 8U)) ||
        ((!frame->is_extended) && (frame->id > 0x7FFU)) ||
        (frame->is_extended && (frame->id > 0x1FFFFFFFU)))
    {
        return false;
    }

    memset(element, 0, sizeof(*element));
    element->id  = frame->is_extended ? frame->id : (frame->id << 18U);
    element->xtd = frame->is_extended ? 1U : 0U;
    element->dlc = dlc;
    element->brs = (frame->is_fd && frame->bit_rate_switch) ? 1U : 0U;
    element->fdf = frame->is_fd ? 1U : 0U;
    memcpy(element->data, frame->data, frame->length);
    return true;
}

static void CANRxElementToFrame(const DL_MCAN_RxBufElement *element, CANFrame *frame)
{
    uint8_t length;

    memset(frame, 0, sizeof(*frame));
    frame->is_extended    = (element->xtd != 0U);
    frame->id             = frame->is_extended ? element->id : ((element->id >> 18U) & 0x7FFU);
    frame->is_fd          = (element->fdf != 0U);
    frame->bit_rate_switch = (element->brs != 0U);
    length = CANDLCToLength(element->dlc);
    if (!frame->is_fd && (length > 8U))
        length = 8U;

    frame->length = length;
    memcpy(frame->data, element->data, frame->length);
}
