/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */

#include <app/lib/filter.h>
#include <math.h>

float ANO_Filter::LPF_1st_Factor_Cal(float sample_rate, float cutoff_freq)
{
    float dt = 1.0f / sample_rate;
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);
    return dt / (RC + dt);
}

void ANO_Filter::LPF_2nd_Factor_Cal(LPF2ndData_t* lpf_data)
{
    // Biquad filter coefficients calculation
    // This matches ANO's implementation for 2nd order LPF
    // Prevent frequency: 30Hz, pass frequency: 500Hz
    lpf_data->b0 = 0.1883633f;
    lpf_data->a1 = 1.023694f;
    lpf_data->a2 = 0.2120577f;
}

float ANO_Filter::CF_Factor_Cal(float sample_rate, float gyro_weight)
{
    float dt = 1.0f / sample_rate;
    return dt / (dt + (1.0f / gyro_weight));
}

Vector3f ANO_Filter::LPF_1st(Vector3f oldData, Vector3f newData, float lpf_factor)
{
    return oldData * (1.0f - lpf_factor) + newData * lpf_factor;
}

Vector3f ANO_Filter::LPF_2nd(LPF2ndData_t* lpf_2nd, Vector3f newData)
{
    Vector3f filtered;

    filtered = newData * lpf_2nd->b0 + lpf_2nd->lastout * lpf_2nd->a1 - lpf_2nd->preout * lpf_2nd->a2;

    // Update state
    lpf_2nd->preout = lpf_2nd->lastout;
    lpf_2nd->lastout = filtered;

    return filtered;
}

Vector3f ANO_Filter::CF_1st(Vector3f gyroData, Vector3f accData, float cf_factor)
{
    return gyroData * cf_factor + accData * (1.0f - cf_factor);
}