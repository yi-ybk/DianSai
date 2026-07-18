/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f407xx.h"
#include "arm_math.h"


#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32
void MatInit(mat *m, uint8_t row, uint8_t col);

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief 返回一块干净的内�?,不过仍然需要强制转�?为你需要的类型
 *
 * @param size 分配大小
 * @return void*
 */
void *zmalloc(size_t size);

// ���ٿ���
float Sqrt(float x);
// ��������
float abs_limit(float num, float Limit);
// �жϷ���λ
float sign(float value);
// ��������
float float_deadband(float Value, float minValue, float maxValue);
// �޷�����
float float_constrain(float Value, float minValue, float maxValue);
// �޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// ѭ���޷�����
float loop_float_constrain(float Input, float minValue, float maxValue);
// �Ƕ� ���޷� 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

/** @brief 按小端字节序读取16位无符号整数 */
uint16_t BytesToUint16LE(const uint8_t bytes[2]);
/** @brief 按大端字节序读取16位无符号整数 */
uint16_t BytesToUint16BE(const uint8_t bytes[2]);
/** @brief 按小端字节序读取16位有符号整数 */
int16_t BytesToInt16LE(const uint8_t bytes[2]);
/** @brief 按大端字节序读取16位有符号整数 */
int16_t BytesToInt16BE(const uint8_t bytes[2]);
/** @brief 按小端字节序读取32位无符号整数 */
uint32_t BytesToUint32LE(const uint8_t bytes[4]);
/** @brief 按大端字节序读取32位无符号整数 */
uint32_t BytesToUint32BE(const uint8_t bytes[4]);
/** @brief 按小端字节序读取32位有符号整数 */
int32_t BytesToInt32LE(const uint8_t bytes[4]);
/** @brief 按大端字节序读取32位有符号整数 */
int32_t BytesToInt32BE(const uint8_t bytes[4]);
/** @brief 按小端字节序将4字节IEEE 754数据转换为float */
float BytesToFloatLE(const uint8_t bytes[4]);
/** @brief 按大端字节序将4字节IEEE 754数据转换为float */
float BytesToFloatBE(const uint8_t bytes[4]);
/** @brief 按小端字节序写入16位无符号整数 */
void Uint16ToBytesLE(uint16_t value, uint8_t bytes[2]);
/** @brief 按大端字节序写入16位无符号整数 */
void Uint16ToBytesBE(uint16_t value, uint8_t bytes[2]);
/** @brief 按小端字节序写入32位无符号整数 */
void Uint32ToBytesLE(uint32_t value, uint8_t bytes[4]);
/** @brief 按大端字节序写入32位无符号整数 */
void Uint32ToBytesBE(uint32_t value, uint8_t bytes[4]);
/** @brief 按小端字节序写入float的IEEE 754数据 */
void FloatToBytesLE(float value, uint8_t bytes[4]);
/** @brief 按大端字节序写入float的IEEE 754数据 */
void FloatToBytesBE(float value, uint8_t bytes[4]);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
