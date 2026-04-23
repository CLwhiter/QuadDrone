/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */
#ifndef APP_LIB_CONTROL_H_
#define APP_LIB_CONTROL_H_

/**
 * @defgroup lib_control Control
 * @ingroup lib
 * @{
 *
 * @brief PID controller library from ANO
 *
 * PID controller implementation matching ANO flight code with scaled integer gains
 */

/**
 * @brief PID controller class from ANO
 */
class ANO_PID {
public:
    // PID gains (scaled integers per D-06 and D-07)
    uint16_t kP, kI, kD;

    // Integral max (scaled integer for EEPROM storage)
    uint32_t imax;

    // Reasonable limits for gains
    static constexpr uint16_t MAX_GAIN = 1000;
    static constexpr uint32_t MAX_IMAX = 1000000;

    // Internal state for integral and previous error
    int32_t integral;
    int32_t prev_error;
    uint16_t prev_dt;

    /**
     * @brief Reset integral term (windup protection)
     */
    void reset_I(void);

    /**
     * @brief Get PID output
     * @param error Current error
     * @param dt Time step in milliseconds
     * @return PID output
     */
    int32_t get_pid(int32_t error, uint16_t dt);

    /**
     * @brief Get PI output
     * @param error Current error
     * @param dt Time step in milliseconds
     * @return PI output
     */
    int32_t get_pi(int32_t error, uint16_t dt);

    /**
     * @brief Get P output
     * @param error Current error
     * @return P output
     */
    int32_t get_p(int32_t error);

    /**
     * @brief Get I output
     * @param error Current error
     * @param dt Time step in milliseconds
     * @return I output
     */
    int32_t get_i(int32_t error, uint16_t dt);

    /**
     * @brief Get D output
     * @param error Current error
     * @param dt Time step in milliseconds
     * @return D output
     */
    int32_t get_d(int32_t error, uint16_t dt);

    /**
     * @brief Validate PID gains are within reasonable bounds
     * @return true if gains are valid, false otherwise
     */
    bool validate_gains() const {
        return (kP <= MAX_GAIN && kI <= MAX_GAIN && kD <= MAX_GAIN &&
                imax <= MAX_IMAX);
    }
};

/**
 * @example Cascaded PID Usage
 * @code
 * // Create outer and inner PID controllers for cascaded control
 * ANO_PID pid_outer;  // Outer loop (e.g., angle control)
 * ANO_PID pid_inner;  // Inner loop (e.g., rate control)
 *
 * // Initialize gains
 * pid_outer.kP = 100;  // Outer P gain
 * pid_outer.kI = 10;   // Outer I gain
 * pid_outer.kD = 1;    // Outer D gain
 * pid_outer.imax = 1000;  // Outer integral limit
 *
 * pid_inner.kP = 50;   // Inner P gain
 * pid_inner.kI = 5;    // Inner I gain
 * pid_inner.kD = 0;    // Inner D gain
 * pid_inner.imax = 500;  // Inner integral limit
 *
 * // Cascaded control loop
 * int32_t angle_error = target_angle - current_angle;
 * int32_t rate_command = pid_outer.get_pid(angle_error, dt);
 * int32_t rate_error = rate_command - current_rate;
 * int32_t motor_output = pid_inner.get_pid(rate_error, dt);
 * @endcode
 */

/** @} */

#endif /* APP_LIB_CONTROL_H_ */