/**
 * @file    mahony_ahrs.c
 * @brief   Mahony六轴姿态解算实现
 */
#include "mahony_ahrs.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define MAHONY_NORM_EPSILON 1.0e-12f
#define MAHONY_RAD_TO_DEG   57.295779513f

static float MahonyConstrain(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}

bool MahonyAhrsInit(MahonyAhrs_t *ahrs, const float initial_quaternion[4])
{
    (void)initial_quaternion;

    if (ahrs == NULL)
        return false;

    ahrs->euler_rad[0] = 0.0f;
    ahrs->euler_rad[1] = 0.0f;
    ahrs->euler_rad[2] = 0.0f;

    return true;
}

bool MahonyAhrsUpdate(MahonyAhrs_t *ahrs,
                      float gx,
                      float gy,
                      float gz,
                      float ax,
                      float ay,
                      float az,
                      float sample_period_s,
                      float proportional_gain,
                      float integral_gain)
{
    float roll;
    float pitch;
    float yaw;
    float roll_g;
    float pitch_g;
    float yaw_g;
    float roll_a;
    float pitch_a;
    float alpha;
    float accel_norm_squared;
    float inverse_norm;

    (void)integral_gain;

    if ((ahrs == NULL) || !(sample_period_s > 0.0f))
    {
        return false;
    }

    roll = ahrs->euler_rad[0];
    pitch = ahrs->euler_rad[1];
    yaw = ahrs->euler_rad[2];

    roll_g = roll + gx * sample_period_s;
    pitch_g = pitch + gy * sample_period_s;
    yaw_g = yaw + gz * sample_period_s;

    accel_norm_squared = ax * ax + ay * ay + az * az;
    if ((accel_norm_squared > MAHONY_NORM_EPSILON) && (proportional_gain > 0.0f))
    {
        inverse_norm = 1.0f / sqrtf(accel_norm_squared);
        ax *= inverse_norm;
        ay *= inverse_norm;
        az *= inverse_norm;

        pitch_a = asinf(MahonyConstrain(-ax, -1.0f, 1.0f));
        roll_a = atan2f(ay, az);

        alpha = proportional_gain;
        if (alpha > 1.0f)
            alpha = 1.0f;
        if (alpha < 0.0f)
            alpha = 0.0f;

        ahrs->euler_rad[0] = alpha * roll_g + (1.0f - alpha) * roll_a;
        ahrs->euler_rad[1] = alpha * pitch_g + (1.0f - alpha) * pitch_a;
        ahrs->euler_rad[2] = yaw_g;
    }
    else
    {
        ahrs->euler_rad[0] = roll_g;
        ahrs->euler_rad[1] = pitch_g;
        ahrs->euler_rad[2] = yaw_g;
    }

    return true;
}

void MahonyAhrsGetEulerDegrees(const MahonyAhrs_t *ahrs,
                               float *roll,
                               float *pitch,
                               float *yaw)
{
    if ((ahrs == NULL) || (roll == NULL) || (pitch == NULL) || (yaw == NULL))
        return;

    *roll = ahrs->euler_rad[0] * MAHONY_RAD_TO_DEG;
    *pitch = ahrs->euler_rad[1] * MAHONY_RAD_TO_DEG;
    *yaw = ahrs->euler_rad[2] * MAHONY_RAD_TO_DEG;
}
