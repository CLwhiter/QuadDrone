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
};

/** @} */

#endif /* APP_LIB_CONTROL_H_ */