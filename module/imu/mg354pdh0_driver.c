/**
 * @file mg354pdh0_driver.c
 * @brief M-G354PDH0 UART初始化与Burst数据帧解析实现
 */
#include "mg354pdh0_driver.h"

#include "FreeRTOS.h"
#include "task.h"

#define MG354PDH0_STARTUP_DELAY_MS       1000U
#define MG354PDH0_COMMAND_DELAY_MS       5U
#define MG354PDH0_COMMAND_TIMEOUT_MS     100U
#define MG354PDH0_COMMAND_LEN            3U
#define MG354PDH0_GYRO_SCALE_RAD_S       0.00027925268f
#define MG354PDH0_ACCEL_SCALE_M_S2       0.00196133f

static int16_t Mg354pdh0ReadInt16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

static bool Mg354pdh0ApplyGyroCalibration(Mg354pdh0GyroCalibration_t *calibration,
                                           ImuVector3f_t *gyro)
{
    if (calibration == NULL)
        return true;

    if (calibration->mode == MG354PDH0_GYRO_CALIBRATION_FIXED_YAW)
    {
        gyro->z -= calibration->gyro_bias.z;
        return true;
    }

    if (calibration->sample_target == 0U)
        return true;

    if (!calibration->calibrated)
    {
        calibration->gyro_sum.x += gyro->x;
        calibration->gyro_sum.y += gyro->y;
        calibration->gyro_sum.z += gyro->z;
        if (calibration->sample_count < calibration->sample_target)
            calibration->sample_count++;

        if (calibration->sample_count < calibration->sample_target)
            return false;

        calibration->gyro_bias.x = calibration->gyro_sum.x /
                                     (float)calibration->sample_count;
        calibration->gyro_bias.y = calibration->gyro_sum.y /
                                     (float)calibration->sample_count;
        calibration->gyro_bias.z = calibration->gyro_sum.z /
                                     (float)calibration->sample_count;
        calibration->calibrated = true;
        return false;
    }

    gyro->x -= calibration->gyro_bias.x;
    gyro->y -= calibration->gyro_bias.y;
    gyro->z -= calibration->gyro_bias.z;
    return true;
}

bool Mg354pdh0DeviceInit(Imu_t *imu, void *context)
{
    static const uint8_t commands[][MG354PDH0_COMMAND_LEN] = {
        {0xFEU, 0x01U, 0x0DU},
        {0x85U, 0x04U, 0x0DU},
        {0x88U, 0x01U, 0x0DU},
        {0x8CU, 0x06U, 0x0DU},
        {0x8DU, 0xF0U, 0x0DU},
        {0x8FU, 0x00U, 0x0DU},
        {0xFEU, 0x00U, 0x0DU},
        {0x83U, 0x01U, 0x0DU},
    };
    uint32_t index;

    (void)context;

    if ((imu == NULL) || (imu->usart == NULL))
        return false;

    vTaskDelay(pdMS_TO_TICKS(MG354PDH0_STARTUP_DELAY_MS));

    for (index = 0U; index < (sizeof(commands) / sizeof(commands[0])); index++)
    {
        if (!USARTSendBlocking(imu->usart,
                               commands[index],
                               MG354PDH0_COMMAND_LEN,
                               MG354PDH0_COMMAND_TIMEOUT_MS))
        {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(MG354PDH0_COMMAND_DELAY_MS));
    }

    return true;
}

uint8_t Mg354pdh0FrameParse(Imu_t *imu,
                            const uint8_t *frame,
                            uint16_t frame_len,
                            void *context)
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    ImuVector3f_t gyro;
    Mg354pdh0GyroCalibration_t *calibration =
        (Mg354pdh0GyroCalibration_t *)context;

    if ((imu == NULL) || (frame == NULL) ||
        (frame_len != MG354PDH0_FRAME_LEN) ||
        (frame[0] != MG354PDH0_FRAME_HEADER) ||
        (frame[MG354PDH0_FRAME_LEN - 1U] != MG354PDH0_FRAME_TAIL))
    {
        return 0U;
    }

    gyro_x  = Mg354pdh0ReadInt16(&frame[5]);
    gyro_y  = Mg354pdh0ReadInt16(&frame[7]);
    gyro_z  = Mg354pdh0ReadInt16(&frame[9]);
    accel_x = Mg354pdh0ReadInt16(&frame[11]);
    accel_y = Mg354pdh0ReadInt16(&frame[13]);
    accel_z = Mg354pdh0ReadInt16(&frame[15]);

    gyro.x = (float)gyro_x * MG354PDH0_GYRO_SCALE_RAD_S;
    gyro.y = (float)gyro_y * MG354PDH0_GYRO_SCALE_RAD_S;
    gyro.z = (float)gyro_z * MG354PDH0_GYRO_SCALE_RAD_S;
    if (!Mg354pdh0ApplyGyroCalibration(calibration, &gyro))
        return 0U;

    imu->data.gyro.x  = gyro.x;
    imu->data.gyro.y  = gyro.y;
    imu->data.gyro.z  = gyro.z;
    imu->data.accel.x = (float)accel_x * MG354PDH0_ACCEL_SCALE_M_S2;
    imu->data.accel.y = (float)accel_y * MG354PDH0_ACCEL_SCALE_M_S2;
    imu->data.accel.z = (float)accel_z * MG354PDH0_ACCEL_SCALE_M_S2;

    return 1U;
}
