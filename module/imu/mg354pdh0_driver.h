/**
 * @file mg354pdh0_driver.h
 * @brief M-G354PDH0 UART驱动接口
 */
#ifndef MG354PDH0_DRIVER_H
#define MG354PDH0_DRIVER_H

#include "imu_driver.h"

#define MG354PDH0_FRAME_LEN     22U
#define MG354PDH0_FRAME_HEADER  0x80U
#define MG354PDH0_FRAME_TAIL    0x0DU

typedef enum
{
    MG354PDH0_GYRO_CALIBRATION_AUTO = 0,
    MG354PDH0_GYRO_CALIBRATION_FIXED_YAW,
} Mg354pdh0GyroCalibrationMode_t;

typedef struct
{
    ImuVector3f_t gyro_sum;
    ImuVector3f_t gyro_bias;
    uint16_t sample_count;
    uint16_t sample_target;
    Mg354pdh0GyroCalibrationMode_t mode;
    bool calibrated;
} Mg354pdh0GyroCalibration_t;

#define MG354PDH0_GYRO_CALIBRATION_DEFAULT(_sample_target) \
    {                                                       \
        .mode = MG354PDH0_GYRO_CALIBRATION_AUTO,           \
        .sample_target = (_sample_target)                  \
    }

#define MG354PDH0_GYRO_CALIBRATION_FIXED_YAW_DEFAULT(_bias_rad_s) \
    {                                                              \
        .mode = MG354PDH0_GYRO_CALIBRATION_FIXED_YAW,              \
        .gyro_bias = {.z = (_bias_rad_s)},                         \
        .calibrated = true                                         \
    }

/**
 * @brief 初始化M-G354PDH0，并配置为125 SPS、16位UART自动输出模式
 * @param imu IMU对象指针
 * @param context 设备初始化上下文，当前未使用
 * @return true初始化成功，false初始化失败
 */
bool Mg354pdh0DeviceInit(Imu_t *imu, void *context);

/**
 * @brief 解析M-G354PDH0的22字节UART Burst数据帧
 * @param imu IMU对象指针
 * @param frame 数据帧指针
 * @param frame_len 数据帧长度
 * @param context 解析上下文，当前未使用
 * @return 1解析成功，0解析失败
 */
uint8_t Mg354pdh0FrameParse(Imu_t *imu,
                            const uint8_t *frame,
                            uint16_t frame_len,
                            void *context);

#endif
