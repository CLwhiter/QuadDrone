/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */
#ifndef APP_LIB_FILTER_H_
#define APP_LIB_FILTER_H_

/**
 * @defgroup lib_filter Filter
 * @ingroup lib
 * @{
 *
 * @brief Digital filter library from ANO
 *
 * Digital filters for sensor data processing including 1st/2nd order LPF and complementary filter
 */

#include <app/lib/math/vector3f.h>

/**
 * @brief Filter state structure for 2nd order LPF
 */
struct LPF2ndData_t {
    float b0;
    float a1;
    float a2;
    Vector3f preout;
    Vector3f lastout;
};

/**
 * @brief Filter class from ANO
 */
class ANO_Filter {
public:
    // Filter coefficients calculation
    static float LPF_1st_Factor_Cal(float sample_rate, float cutoff_freq);
    static void LPF_2nd_Factor_Cal(LPF2ndData_t* lpf_data);
    static float CF_Factor_Cal(float sample_rate, float gyro_weight);

    // Filter application
    Vector3f LPF_1st(Vector3f oldData, Vector3f newData, float lpf_factor);
    Vector3f LPF_2nd(LPF2ndData_t* lpf_2nd, Vector3f newData);
    Vector3f CF_1st(Vector3f gyroData, Vector3f accData, float cf_factor);
};

/** @} */

#endif /* APP_LIB_FILTER_H_ */