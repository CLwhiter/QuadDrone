/*
 * Copyright (c) 2025 ANO Open Source Flight Control
 * SPDX-License-Identifier: Apache-2.0
 */

#include "math/math_utils.h"

float safe_asin(float v)
{
    if (v >= 1.0f) {
        return M_PI/2.0f;
    }
    if (v <= -1.0f) {
        return -M_PI/2.0f;
    }
    return asinf(v);
}

// Float constraint
float constrain_float(float amt, float low, float high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

// 16-bit signed constraint
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

// 16-bit unsigned constraint
uint16_t constrain_uint16(uint16_t amt, uint16_t low, uint16_t high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

// 32-bit signed constraint
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

// Degrees to radians
float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

// Radians to degrees
float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

// Square
float sq(float v)
{
    return v * v;
}

// 2D Pythagorean theorem
float pythagorous2(float a, float b)
{
    return sqrtf(sq(a) + sq(b));
}

// 3D Pythagorean theorem
float pythagorous3(float a, float b, float c)
{
    return sqrtf(sq(a) + sq(b) + sq(c));
}

// 4D Pythagorean theorem
float pythagorous4(float a, float b, float c, float d)
{
    return sqrtf(sq(a) + sq(b) + sq(c) + sq(d));
}