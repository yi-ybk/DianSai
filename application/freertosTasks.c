#include "freertosTasks.h"
#include "usart.h"

#define IMU_TASK_PERIOD_MS 1U

Imu_t imu = IMU_OBJECT_DEFAULT;

Imu_t *ImuGetObject(void)
{
    return &imu;
}

void ImuParseTask(void *argument)
{
    Imu_t *imu_obj = (Imu_t *)argument;
    ImuInitConfig_t imu_config = {
        .recv_buff_size = IMU_UART_DMA_RX_BUFFER_LEN,
        .usart_handle = &huart1,
        .parser = NULL,
        .parser_context = NULL,
    };

    if (imu_obj == NULL)
        imu_obj = &imu;

    if (imu_obj->init != NULL)
        (void)imu_obj->init(imu_obj, &imu_config);

    for (;;)
    {
        if (imu_obj->process != NULL)
            imu_obj->process(imu_obj);

        osDelay(IMU_TASK_PERIOD_MS);
    }
}
