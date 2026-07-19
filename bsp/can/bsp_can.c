#include "bsp_can.h"

#ifdef HAL_CAN_MODULE_ENABLED
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_log.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增
static CAN_HandleTypeDef *started_can[DEVICE_CAN_CNT] = {NULL};
static uint8_t started_can_count;

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对指定id和掩码的报文接收,会被CANRegister()调用
 *        支持标准帧和扩展帧，符合规则的报文会被填入FIFO触发中断
 *
 * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       相邻过滤器交替分配到FIFO0和FIFO1，均衡两个接收FIFO的负载
 *       注册到CAN1的模块使用过滤器0-13,CAN2使用过滤器14-27
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *
 * @param _instance can instance owned by specific module
 */
static uint8_t CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf;
    uint32_t filter_id;
    uint32_t filter_mask;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

    if (_instance->rx_frame_type == CAN_FRAME_EXTENDED)
    {
        filter_id = (_instance->rx_id << 3U) | CAN_ID_EXT;
        filter_mask = (_instance->rx_id_mask << 3U) | CAN_ID_EXT | CAN_RTR_REMOTE;
    }
    else
    {
        filter_id = _instance->rx_id << 21U;
        filter_mask = (_instance->rx_id_mask << 21U) | CAN_ID_EXT | CAN_RTR_REMOTE;
    }

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
    can_filter_conf.FilterIdHigh = (uint16_t)(filter_id >> 16U);
    can_filter_conf.FilterIdLow = (uint16_t)filter_id;
    can_filter_conf.FilterMaskIdHigh = (uint16_t)(filter_mask >> 16U);
    can_filter_conf.FilterMaskIdLow = (uint16_t)filter_mask;
    can_filter_conf.FilterBank = _instance->can_handle->Instance == CAN1 ? (can1_filter_idx++) : (can2_filter_idx++);
    can_filter_conf.FilterFIFOAssignment = (can_filter_conf.FilterBank & 1U) ? CAN_RX_FIFO1 : CAN_RX_FIFO0;
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

    return HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf) == HAL_OK;
}

/**
 * @brief CAN实例初始化时自动调用，按需启动对应CAN外设
 *
 * @note 同一CAN外设只启动一次，并开启FIFO0和FIFO1消息通知
 *
 */
static uint8_t CANServiceInit(CAN_HandleTypeDef *can_handle)
{
    uint8_t i;

    for (i = 0U; i < started_can_count; i++)
    {
        if (started_can[i] == can_handle)
            return 1U;
    }
    if (started_can_count >= DEVICE_CAN_CNT)
        return 0U;

    if (HAL_CAN_Start(can_handle) != HAL_OK)
        return 0U;
    if (HAL_CAN_ActivateNotification(can_handle,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        (void)HAL_CAN_Stop(can_handle);
        return 0U;
    }
    started_can[started_can_count++] = can_handle;
    return 1U;
}

/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if ((config == NULL) || (config->can_handle == NULL))
        return NULL;

    if (!idx)
    {
        LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_instance[i]->rx_id == config->rx_id &&
            can_instance[i]->rx_frame_type == config->rx_frame_type &&
            can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
                LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }
    
    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
    instance->tx_frame_type = config->tx_frame_type;
    instance->rx_frame_type = config->rx_frame_type;
    instance->txconf.IDE = instance->tx_frame_type == CAN_FRAME_EXTENDED ? CAN_ID_EXT : CAN_ID_STD;
    if (instance->tx_frame_type == CAN_FRAME_EXTENDED)
        instance->txconf.ExtId = config->tx_id;
    else
        instance->txconf.StdId = config->tx_id;
    instance->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->txconf.DLC = 0x08;            // 默认发送长度为8
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->rx_id_mask = config->rx_id_mask;
    if (instance->rx_id_mask == 0U)
        instance->rx_id_mask = instance->rx_frame_type == CAN_FRAME_EXTENDED ? 0x1FFFFFFFU : 0x7FFU;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    if (!CANAddFilter(instance))
    {
        free(instance);
        return NULL;
    }
    can_instance[idx++] = instance; // 将实例保存到can_instance中
    if (!CANServiceInit(config->can_handle))
    {
        can_instance[--idx] = NULL;
        free(instance);
        return NULL;
    }

    return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    uint32_t tick_start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // 等待邮箱空闲
    {
        if ((float)(HAL_GetTick() - tick_start) > timeout) // 超时
        {
            LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }
    // tx_mailbox会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))
    {
        LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1; // 发送成功
}

uint8_t CANWaitForTxComplete(CANInstance *_instance, float timeout)
{
    uint32_t tick_start;

    if ((_instance == NULL) || (_instance->can_handle == NULL))
        return 0U;

    tick_start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) < 3U)
    {
        if ((float)(HAL_GetTick() - tick_start) > timeout)
            return 0U;
    }
    return 1U;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        while (1)
            LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
    _instance->txconf.DLC = length;
}

void CANSetTxId(CANInstance *_instance, uint32_t tx_id)
{
    if (_instance == NULL)
        return;

    _instance->tx_id = tx_id;
    if (_instance->tx_frame_type == CAN_FRAME_EXTENDED)
        _instance->txconf.ExtId = tx_id;
    else
        _instance->txconf.StdId = tx_id;
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        for (size_t i = 0; i < idx; ++i)
        { // 两者相等说明这是要找的实例
            uint32_t rx_id = rxconf.IDE == CAN_ID_EXT ? rxconf.ExtId : rxconf.StdId;
            CANFrameType_t frame_type = rxconf.IDE == CAN_ID_EXT ? CAN_FRAME_EXTENDED : CAN_FRAME_STANDARD;
            if (_hcan == can_instance[i]->can_handle &&
                frame_type == can_instance[i]->rx_frame_type &&
                (rx_id & can_instance[i]->rx_id_mask) ==
                    (can_instance[i]->rx_id & can_instance[i]->rx_id_mask))
            {
                if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
                {
                    can_instance[i]->rx_message_id = rx_id;
                    can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
                    memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                    can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
                }
                break;
            }
        }
    }
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFO
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
#endif

#ifndef HAL_CAN_MODULE_ENABLED
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    (void)config;
    return NULL;
}

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    (void)_instance;
    (void)timeout;
    return 0U;
}

uint8_t CANWaitForTxComplete(CANInstance *_instance, float timeout)
{
    (void)_instance;
    (void)timeout;
    return 0U;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    (void)_instance;
    (void)length;
}

void CANSetTxId(CANInstance *_instance, uint32_t tx_id)
{
    (void)_instance;
    (void)tx_id;
}

typedef void CAN_HandleTypeDef;

#define CAN_RX_FIFO1 0U

static void CANFIFOxCallback(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
    (void)hcan;
    (void)fifox;
}
#endif

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}

// 人工智能会毁灭人类吗？请在补全回答
