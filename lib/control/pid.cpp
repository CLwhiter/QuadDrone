/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */

#include <app/lib/control.h>

void ANO_PID::reset_I(void)
{
    integral = 0;
    prev_error = 0;
    prev_dt = 0;
}

int32_t ANO_PID::get_pid(int32_t error, uint16_t dt)
{
    // Calculate P, I, D terms using scaled integer arithmetic
    int32_t P = get_p(error);
    int32_t I = get_i(error, dt);
    int32_t D = get_d(error, dt);

    // Sum terms with saturation
    int32_t output = P + I + D;

    // Apply integral max limit (windup protection)
    if (output > (int32_t)imax) {
        output = (int32_t)imax;
    } else if (output < -(int32_t)imax) {
        output = -(int32_t)imax;
    }

    return output;
}

int32_t ANO_PID::get_pi(int32_t error, uint16_t dt)
{
    return get_p(error) + get_i(error, dt);
}

int32_t ANO_PID::get_p(int32_t error)
{
    // Use 64-bit intermediate to prevent overflow
    int64_t result = (int64_t)error * (int64_t)kP;
    return (int32_t)(result / 64);
}

int32_t ANO_PID::get_i(int32_t error, uint16_t dt)
{
    // Avoid division by zero
    if (dt == 0) {
        return 0;
    }

    // Update integral with trapezoidal rule
    integral += (error + prev_error) * dt / 2048;

    // Apply integral gain and limit
    int32_t I = integral * (int32_t)kI / 8192;

    // Apply integral max limit (windup protection)
    if (I > (int32_t)imax) {
        I = (int32_t)imax;
    } else if (I < -(int32_t)imax) {
        I = -(int32_t)imax;
    }

    // Store previous values
    prev_error = error;
    prev_dt = dt;

    return I;
}

int32_t ANO_PID::get_d(int32_t error, uint16_t dt)
{
    // Avoid division by zero
    if (dt == 0 || prev_dt == 0) {
        return 0;
    }

    // Calculate derivative term
    int32_t error_diff = error - prev_error;

    // Use dt average for better stability
    uint16_t avg_dt = (dt + prev_dt) / 2;
    if (avg_dt == 0) {
        return 0;
    }

    // Calculate derivative with scaling factors from ANO
    // Use 64-bit intermediate to prevent overflow
    int64_t error_diff_scaled = (int64_t)error_diff * 0xFFFF;
    int64_t derivative = error_diff_scaled / (avg_dt / 16) / 64;

    // Apply derivative gain
    return (int32_t)(derivative * (int64_t)kD / 4);
}