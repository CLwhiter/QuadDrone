/*
 * Copyright (c) 2025 ANO Open Source Flight Control
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_LIB_MATH_QUATERNION_H_
#define APP_LIB_MATH_QUATERNION_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <stdbool.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "math/vector3.h"

// Quaternion with elements of type T
template <typename T>
class Quaternion {
public:
    static_assert(std::is_floating_point<T>::value, "Quaternion only supports floating-point types");
    T w, x, y, z;

    // Default constructor (identity quaternion)
    Quaternion<T>() {
        w = 1.0f;
        x = y = z = 0.0f;
    }

    // Parameterized constructor
    Quaternion<T>(const T w0, const T x0, const T y0, const T z0) : w(w0), x(x0), y(y0), z(z0) {
    }

    // Constructor from vector and scalar
    Quaternion<T>(const T scalar, const Vector3<T> &vector) : w(scalar), x(vector.x), y(vector.y), z(vector.z) {
    }

    // Constructor from array
    Quaternion<T>(const T q[4]) : w(q[0]), x(q[1]), y(q[2]), z(q[3]) {
    }

    // Set quaternion values
    void operator () (const T w0, const T x0, const T y0, const T z0)
    {
        w = w0; x = x0; y = y0; z = z0;
    }

    // Check if quaternions are equal
    bool operator == (const Quaternion<T> &q)
    {
        return (w==q.w && x==q.x && y==q.y && z==q.z);
    }

    // Check if quaternions are not equal
    bool operator != (const Quaternion<T> &q)
    {
        return (w!=q.w || x!=q.x || y!=q.y || z!=q.z);
    }

    // Unary negation
    Quaternion<T> operator - (void) const
    {
        return Quaternion<T>(-w, -x, -y, -z);
    }

    // Addition
    Quaternion<T> operator + (const Quaternion<T> &q) const
    {
        return Quaternion<T>(w+q.w, x+q.x, y+q.y, z+q.z);
    }
    Quaternion<T> &operator += (const Quaternion<T> &q)
    {
        return *this = *this + q;
    }

    // Subtraction
    Quaternion<T> operator - (const Quaternion<T> &q) const
    {
        return Quaternion<T>(w-q.w, x-q.x, y-q.y, z-q.z);
    }
    Quaternion<T> &operator -= (const Quaternion<T> &q)
    {
        return *this = *this - q;
    }

    // Scalar multiplication
    Quaternion<T> operator * (const T num) const
    {
        return Quaternion<T>(w*num, x*num, y*num, z*num);
    }
    Quaternion<T> &operator *= (const T num)
    {
        return *this = *this * num;
    }
    Quaternion<T> operator / (const T num) const
    {
        return Quaternion<T>(w/num, x/num, y/num, z/num);
    }
    Quaternion<T> &operator /= (const T num)
    {
        return *this = *this / num;
    }

    // Quaternion multiplication
    Quaternion<T> operator *(const Quaternion<T> &q) const;

    // Get conjugate
    Quaternion<T> conjugate(void) const
    {
        return Quaternion<T>(w, -x, -y, -z);
    }

    // Get magnitude
    T magnitude(void) const
    {
        return sqrtf(w*w + x*x + y*y + z*z);
    }

    // Get squared magnitude
    T magnitude_squared(void) const
    {
        return w*w + x*x + y*y + z*z;
    }

    // Normalize in place
    void normalize(void)
    {
        T mag = magnitude();
        if (mag > 1e-6f) {  // Use small epsilon instead of zero
            w /= mag;
            x /= mag;
            y /= mag;
            z /= mag;
        } else {
            identity();  // Fallback to identity quaternion
        }
    }

    // Get normalized quaternion
    Quaternion<T> normalized(void) const
    {
        Quaternion<T> q = *this;
        q.normalize();
        return q;
    }

    // Set to identity
    void identity(void) {
        w = 1.0f;
        x = y = z = 0.0f;
    }

    // Check if quaternion has abnormal values
    bool is_nan(void)
    {
        return isnan(w) || isnan(x) || isnan(y) || isnan(z);
    }

    // Convert to rotation matrix
    void to_matrix(Matrix3<T> &m) const;

    // Convert from rotation matrix
    void from_matrix(const Matrix3<T> &m);

    // Convert from Euler angles
    void from_euler(const Vector3<T> &euler);

    // Convert to Euler angles
    void to_euler(float *roll, float *pitch, float *yaw) const;

    // Get yaw, pitch, roll angles
    Vector3<T> get_euler(void) const;

    // Rotate a vector by this quaternion
    Vector3<T> rotate_vector(const Vector3<T> &v) const;

    // Inverse rotate a vector by this quaternion
    Vector3<T> inverse_rotate_vector(const Vector3<T> &v) const;

    // Normalize if needed (only normalize if magnitude significantly different from 1)
    void normalize_if_needed(void);

    // Fast inverse
    Quaternion<T> inverse(void) const;

    // Spherical linear interpolation
    Quaternion<T> slerp(const Quaternion<T> &q, const T t) const;

    // Compute angular velocity from quaternion derivative
    Vector3<T> get_gyro(const Quaternion<T> &dq) const;

    // Integrate angular velocity
    void integrate_gyro(const Vector3<T> &gyro, const T dt);

    // Compute quaternion derivative from angular velocity
    Quaternion<T> get_derivative(const Vector3<T> &gyro) const;

    // Set from two vectors (creates rotation from v1 to v2)
    void set_from_two_vectors(const Vector3<T> &v1, const Vector3<T> &v2);

    // Check if valid unit quaternion
    bool is_valid(void) const;

    // Get dot product
    T dot(const Quaternion<T> &q) const {
        return w*q.w + x*q.x + y*q.y + z*q.z;
    }

    // Get angle to another quaternion
    T angle_to(const Quaternion<T> &q) const;

    // Get shortest angle to another quaternion
    T shortest_angle_to(const Quaternion<T> &q) const;
};

// Type aliases
typedef Quaternion<float>                   Quaternionf;
typedef Quaternion<double>                  Quaterniond;

// Helper functions
float normalize_angle(float angle);
float normalize_half_angle(float angle);
float wrap_2pi(float angle);
float wrap_pi(float angle);

// Fast math approximations
float fast_sqrt(float x);
float fast_inv_sqrt(float x);

#ifdef __cplusplus
}
#endif

#endif /* APP_LIB_MATH_QUATERNION_H_ */