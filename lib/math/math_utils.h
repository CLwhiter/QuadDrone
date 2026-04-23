/*
 * Copyright (c) 2025 ANO Open Source Flight Control
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_LIB_MATH_UTILS_H_
#define APP_LIB_MATH_UTILS_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M_PI 3.141592653f
#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f

float safe_asin(float v);

// Float constraint
float constrain_float(float amt, float low, float high);

// 16-bit signed constraint
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);

// 16-bit unsigned constraint
uint16_t constrain_uint16(uint16_t amt, uint16_t low, uint16_t high);

// 32-bit signed constraint
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);

// Degrees to radians
float radians(float deg);

// Radians to degrees
float degrees(float rad);

// Square
float sq(float v);

// 2D Pythagorean theorem
float pythagorous2(float a, float b);

// 3D Pythagorean theorem
float pythagorous3(float a, float b, float c);

// 4D Pythagorean theorem
float pythagorous4(float a, float b, float c, float d);

#ifdef __cplusplus
}
#endif

#endif /* APP_LIB_MATH_UTILS_H_ */