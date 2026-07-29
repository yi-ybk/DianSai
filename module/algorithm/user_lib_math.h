/**
 ******************************************************************************
 * @file    user_lib_math.h
 * @brief   通用数值辅助函数接口
 ******************************************************************************
 */
#ifndef USER_LIB_MATH_H
#define USER_LIB_MATH_H

float abs_limit(float num, float limit);
float float_constrain(float value, float min_value, float max_value);
float angle_wrap_180(float angle);

#endif
