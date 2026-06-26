#include "imu.h"
#include "usart.h"

#define IMU_TASK_PERIOD_MS 1

Imu_t imu0 = {                                           
        IMU_OBJECT_DEFAULT,

        .protocol_frame_len    = 10,
        .protocol_header_bytes = (const uint8_t[]){0xAA, 0x55},
        .protocol_header_len   = 2,
        .protocol_tail_bytes   = (const uint8_t[]){0x0D, 0x0A},
        .protocol_tail_len     = 2,

        .init_config = {
            .recv_buff_size = IMU_UART_DMA_RX_BUFFER_LEN,
            .usart_handle   = &huart1,
            .parser         = Imu0FrameParse,
            .parser_context = NULL,
        },
};

void ImuParseTask(void *argument)
{
    Imu_t *imu = (Imu_t *)argument;

    if (imu == NULL)
        while (1);

    if (imu->init != NULL)
        (void)imu->init(imu, &imu->init_config);

    for (;;)
    {
        if (imu->process != NULL)
            imu->process(imu);

        osDelay(IMU_TASK_PERIOD_MS);
    }
}
