/**
 * @file    mahony_ahrs.h
 * @brief   Mahony六轴姿态解算接口
 */
#pragma once

#include <stdbool.h>

typedef struct
{
    float quaternion[4];
    float integral_feedback[3];
} MahonyAhrs_t;

bool MahonyAhrsInit(MahonyAhrs_t *ahrs, const float initial_quaternion[4]);

bool MahonyAhrsUpdate(MahonyAhrs_t *ahrs,
                      float gx,
                      float gy,
                      float gz,
                      float ax,
                      float ay,
                      float az,
                      float sample_period_s,
                      float proportional_gain,
                      float integral_gain);

void MahonyAhrsGetEulerDegrees(const MahonyAhrs_t *ahrs,
                               float *roll,
                               float *pitch,
                               float *yaw);
