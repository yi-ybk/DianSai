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
    float norm_squared;
    float inverse_norm;

    if ((ahrs == NULL) || (initial_quaternion == NULL))
        return false;

    norm_squared = initial_quaternion[0] * initial_quaternion[0] +
                   initial_quaternion[1] * initial_quaternion[1] +
                   initial_quaternion[2] * initial_quaternion[2] +
                   initial_quaternion[3] * initial_quaternion[3];
    if (!(norm_squared > MAHONY_NORM_EPSILON))
        return false;

    inverse_norm = 1.0f / sqrtf(norm_squared);
    for (uint8_t i = 0U; i < 4U; ++i)
        ahrs->quaternion[i] = initial_quaternion[i] * inverse_norm;

    for (uint8_t i = 0U; i < 3U; ++i)
        ahrs->integral_feedback[i] = 0.0f;

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
    float q0;
    float q1;
    float q2;
    float q3;
    float integral_x;
    float integral_y;
    float integral_z;
    float accel_norm_squared;
    float quaternion_norm_squared;
    float inverse_norm;
    float half_gravity_x;
    float half_gravity_y;
    float half_gravity_z;
    float half_error_x;
    float half_error_y;
    float half_error_z;
    float half_sample_period;
    float next_q0;
    float next_q1;
    float next_q2;
    float next_q3;

    if ((ahrs == NULL) || !(sample_period_s > 0.0f) ||
        !(proportional_gain >= 0.0f) || !(integral_gain >= 0.0f))
    {
        return false;
    }

    q0 = ahrs->quaternion[0];
    q1 = ahrs->quaternion[1];
    q2 = ahrs->quaternion[2];
    q3 = ahrs->quaternion[3];
    integral_x = ahrs->integral_feedback[0];
    integral_y = ahrs->integral_feedback[1];
    integral_z = ahrs->integral_feedback[2];

    accel_norm_squared = ax * ax + ay * ay + az * az;
    if (accel_norm_squared > MAHONY_NORM_EPSILON)
    {
        inverse_norm = 1.0f / sqrtf(accel_norm_squared);
        ax *= inverse_norm;
        ay *= inverse_norm;
        az *= inverse_norm;

        half_gravity_x = q1 * q3 - q0 * q2;
        half_gravity_y = q0 * q1 + q2 * q3;
        half_gravity_z = q0 * q0 - 0.5f + q3 * q3;

        half_error_x = ay * half_gravity_z - az * half_gravity_y;
        half_error_y = az * half_gravity_x - ax * half_gravity_z;
        half_error_z = ax * half_gravity_y - ay * half_gravity_x;

        if (integral_gain > 0.0f)
        {
            integral_x += 2.0f * integral_gain * half_error_x * sample_period_s;
            integral_y += 2.0f * integral_gain * half_error_y * sample_period_s;
            integral_z += 2.0f * integral_gain * half_error_z * sample_period_s;
            gx += integral_x;
            gy += integral_y;
            gz += integral_z;
        }
        else
        {
            integral_x = 0.0f;
            integral_y = 0.0f;
            integral_z = 0.0f;
        }

        gx += 2.0f * proportional_gain * half_error_x;
        gy += 2.0f * proportional_gain * half_error_y;
        gz += 2.0f * proportional_gain * half_error_z;
    }

    half_sample_period = 0.5f * sample_period_s;
    next_q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_sample_period;
    next_q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * half_sample_period;
    next_q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * half_sample_period;
    next_q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * half_sample_period;

    quaternion_norm_squared = next_q0 * next_q0 + next_q1 * next_q1 +
                              next_q2 * next_q2 + next_q3 * next_q3;
    if (!(quaternion_norm_squared > MAHONY_NORM_EPSILON))
        return false;

    inverse_norm = 1.0f / sqrtf(quaternion_norm_squared);
    ahrs->quaternion[0] = next_q0 * inverse_norm;
    ahrs->quaternion[1] = next_q1 * inverse_norm;
    ahrs->quaternion[2] = next_q2 * inverse_norm;
    ahrs->quaternion[3] = next_q3 * inverse_norm;
    ahrs->integral_feedback[0] = integral_x;
    ahrs->integral_feedback[1] = integral_y;
    ahrs->integral_feedback[2] = integral_z;

    return true;
}

void MahonyAhrsGetEulerDegrees(const MahonyAhrs_t *ahrs,
                               float *roll,
                               float *pitch,
                               float *yaw)
{
    float q0;
    float q1;
    float q2;
    float q3;
    float pitch_sine;

    if ((ahrs == NULL) || (roll == NULL) || (pitch == NULL) || (yaw == NULL))
        return;

    q0 = ahrs->quaternion[0];
    q1 = ahrs->quaternion[1];
    q2 = ahrs->quaternion[2];
    q3 = ahrs->quaternion[3];

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * MAHONY_RAD_TO_DEG;
    pitch_sine = MahonyConstrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f);
    *pitch = asinf(pitch_sine) * MAHONY_RAD_TO_DEG;
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * MAHONY_RAD_TO_DEG;
}
