/*
 * Copyright (c) 2025 ANO Open Source Flight Control
 * SPDX-License-Identifier: Apache-2.0
 */

#include "math/quaternion.h"
#include "math/math_utils.h"

#define HALF_PI 1.57079632679f

// Quaternion multiplication
template <typename T>
Quaternion<T> Quaternion<T>::operator *(const Quaternion<T> &q) const
{
    return Quaternion<T>(w*q.w - x*q.x - y*q.y - z*q.z,
                         w*q.x + x*q.w + y*q.z - z*q.y,
                         w*q.y - x*q.z + y*q.w + z*q.x,
                         w*q.z + x*q.y - y*q.x + z*q.w);
}

// Convert to rotation matrix
template <typename T>
void Quaternion<T>::to_matrix(Matrix3<T> &m) const
{
    T wx, wy, wz, xx, xy, xz, yy, yz, zz;

    wx = w * x;
    wy = w * y;
    wz = w * z;
    xx = x * x;
    xy = x * y;
    xz = x * z;
    yy = y * y;
    yz = y * z;
    zz = z * z;

    m.a.x = 1 - 2 * (yy + zz);
    m.a.y = 2 * (xy - wz);
    m.a.z = 2 * (xz + wy);

    m.b.x = 2 * (xy + wz);
    m.b.y = 1 - 2 * (xx + zz);
    m.b.z = 2 * (yz - wx);

    m.c.x = 2 * (xz - wy);
    m.c.y = 2 * (yz + wx);
    m.c.z = 1 - 2 * (xx + yy);
}

// Convert from rotation matrix
template <typename T>
void Quaternion<T>::from_matrix(const Matrix3<T> &m)
{
    T trace = m.a.x + m.b.y + m.c.z;
    T s;

    if (trace > 0) {
        s = sqrtf(trace + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (m.c.y - m.b.z) / s;
        y = (m.a.z - m.c.x) / s;
        z = (m.b.x - m.a.y) / s;
    } else if (m.a.x > m.b.y && m.a.x > m.c.z) {
        s = sqrtf(1.0f + m.a.x - m.b.y - m.c.z) * 2.0f;
        w = (m.c.y - m.b.z) / s;
        x = 0.25f * s;
        y = (m.a.z + m.c.x) / s;
        z = (m.b.x + m.a.y) / s;
    } else if (m.b.y > m.c.z) {
        s = sqrtf(1.0f + m.b.y - m.a.x - m.c.z) * 2.0f;
        w = (m.a.z - m.c.x) / s;
        x = (m.a.z + m.c.x) / s;
        y = 0.25f * s;
        z = (m.b.x + m.a.y) / s;
    } else {
        s = sqrtf(1.0f + m.c.z - m.a.x - m.b.y) * 2.0f;
        w = (m.b.x - m.a.y) / s;
        x = (m.b.x + m.a.y) / s;
        y = (m.a.z + m.c.x) / s;
        z = 0.25f * s;
    }
}

// Convert from Euler angles
template <typename T>
void Quaternion<T>::from_euler(const Vector3<T> &euler)
{
    T half_roll = euler.x * 0.5f;
    T half_pitch = euler.y * 0.5f;
    T half_yaw = euler.z * 0.5f;

    T cr = cosf(half_roll);
    T sr = sinf(half_roll);
    T cp = cosf(half_pitch);
    T sp = sinf(half_pitch);
    T cy = cosf(half_yaw);
    T sy = sinf(half_yaw);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

// Convert to Euler angles
template <typename T>
void Quaternion<T>::to_euler(float *roll, float *pitch, float *yaw) const
{
    T test = x * y + z * w;

    if (test > 0.499f) {
        // Singularity at north pole
        if (pitch != NULL) {
            *pitch = HALF_PI;
        }
        if (roll != NULL) {
            *roll = 2.0f * atan2f(x, w);
        }
        if (yaw != NULL) {
            *yaw = 0.0f;
        }
        return;
    }

    if (test < -0.499f) {
        // Singularity at south pole
        if (pitch != NULL) {
            *pitch = -HALF_PI;
        }
        if (roll != NULL) {
            *roll = -2.0f * atan2f(x, w);
        }
        if (yaw != NULL) {
            *yaw = 0.0f;
        }
        return;
    }

    T sqx = x * x;
    T sqy = y * y;
    T sqz = z * z;

    if (pitch != NULL) {
        *pitch = asinf(2.0f * test);
    }
    if (roll != NULL) {
        *roll = atan2f(2.0f * (y*w - z*x), 1.0f - 2.0f * (sqy + sqz));
    }
    if (yaw != NULL) {
        *yaw = atan2f(2.0f * (x*w - z*y), 1.0f - 2.0f * (sqx + sqy));
    }
}

// Get yaw, pitch, roll angles
template <typename T>
Vector3<T> Quaternion<T>::get_euler(void) const
{
    Vector3<T> euler;
    to_euler(&euler.x, &euler.y, &euler.z);
    return euler;
}

// Rotate a vector by this quaternion
template <typename T>
Vector3<T> Quaternion<T>::rotate_vector(const Vector3<T> &v) const
{
    Quaternion<T> qv(0, v.x, v.y, v.z);
    Quaternion<T> q_conj = conjugate();
    Quaternion<T> result = (*this) * qv * q_conj;
    return Vector3<T>(result.x, result.y, result.z);
}

// Inverse rotate a vector by this quaternion
template <typename T>
Vector3<T> Quaternion<T>::inverse_rotate_vector(const Vector3<T> &v) const
{
    Quaternion<T> qv(0, v.x, v.y, v.z);
    Quaternion<T> q_conj = conjugate();
    Quaternion<T> result = q_conj * qv * (*this);
    return Vector3<T>(result.x, result.y, result.z);
}

// Normalize if needed
template <typename T>
void Quaternion<T>::normalize_if_needed(void)
{
    T mag_sq = magnitude_squared();
    if (mag_sq > 1.01f || mag_sq < 0.99f) {
        normalize();
    }
}

// Fast inverse
template <typename T>
Quaternion<T> Quaternion<T>::inverse(void) const
{
    T mag_sq = magnitude_squared();
    if (mag_sq > 0.0f) {
        return conjugate() / mag_sq;
    }
    return *this;
}

// Spherical linear interpolation
template <typename T>
Quaternion<T> Quaternion<T>::slerp(const Quaternion<T> &q, const T t) const
{
    Quaternion<T> q1 = *this;
    Quaternion<T> q2 = q;

    // Ensure we take the shortest path
    T dot = q1.dot(q2);
    if (dot < 0.0f) {
        q2 = -q2;
        dot = -dot;
    }

    // Check if quaternions are very close
    if (dot > 0.9995f) {
        // Linear interpolation for very close quaternions
        Quaternion<T> result = q1 + (q2 - q1) * t;
        result.normalize();
        return result;
    }

    // Standard SLERP
    T theta_0 = acosf(constrain_float(dot, -1.0f, 1.0f));
    T theta = theta_0 * t;
    T sin_theta = sinf(theta);
    T sin_theta_0 = sinf(theta_0);

    T s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
    T s1 = sin_theta / sin_theta_0;

    Quaternion<T> result = q1 * s0 + q2 * s1;
    result.normalize();
    return result;
}

// Compute angular velocity from quaternion derivative
template <typename T>
Vector3<T> Quaternion<T>::get_gyro(const Quaternion<T> &dq) const
{
    Vector3<T> gyro;
    gyro.x = 2.0f * (dq.w * x - dq.x * w - dq.y * z + dq.z * y);
    gyro.y = 2.0f * (dq.w * y + dq.x * z - dq.y * w - dq.z * x);
    gyro.z = 2.0f * (dq.w * z - dq.x * y + dq.y * x - dq.z * w);
    return gyro;
}

// Integrate angular velocity
template <typename T>
void Quaternion<T>::integrate_gyro(const Vector3<T> &gyro, const T dt)
{
    Quaternion<T> dq;
    dq.w = 1.0f;
    dq.x = 0.5f * gyro.x * dt;
    dq.y = 0.5f * gyro.y * dt;
    dq.z = 0.5f * gyro.z * dt;

    (*this) = (*this) * dq;
    normalize_if_needed();
}

// Compute quaternion derivative from angular velocity
template <typename T>
Quaternion<T> Quaternion<T>::get_derivative(const Vector3<T> &gyro) const
{
    Quaternion<T> dq;
    dq.w = -0.5f * (x * gyro.x + y * gyro.y + z * gyro.z);
    dq.x = 0.5f * (w * gyro.x + z * gyro.y - y * gyro.z);
    dq.y = 0.5f * (w * gyro.y + x * gyro.z - z * gyro.x);
    dq.z = 0.5f * (w * gyro.z + y * gyro.x - x * gyro.y);
    return dq;
}

// Set from two vectors
template <typename T>
void Quaternion<T>::set_from_two_vectors(const Vector3<T> &v1, const Vector3<T> &v2)
{
    // Normalize vectors
    Vector3<T> v1_norm = v1.normalized();
    Vector3<T> v2_norm = v2.normalized();

    // Calculate rotation axis (cross product)
    Vector3<T> axis = v1_norm % v2_norm;
    T axis_len = axis.length();

    if (axis_len < 1e-6f) {
        // Vectors are parallel or anti-parallel
        if (v1_norm.dot(v2_norm) < 0.0f) {
            // 180 degree rotation - need to find an arbitrary perpendicular axis
            // Find smallest component of v1
            T abs_x = fabsf(v1_norm.x);
            T abs_y = fabsf(v1_norm.y);
            T abs_z = fabsf(v1_norm.z);

            if (abs_x < abs_y && abs_x < abs_z) {
                axis = Vector3<T>(0, -v1_norm.z, v1_norm.y).normalized();
            } else if (abs_y < abs_z) {
                axis = Vector3<T>(-v1_norm.z, 0, v1_norm.x).normalized();
            } else {
                axis = Vector3<T>(-v1_norm.y, v1_norm.x, 0).normalized();
            }
        } else {
            // Vectors are parallel - identity quaternion
            identity();
            return;
        }
    } else {
        axis = axis / axis_len;
    }

    // Calculate rotation angle
    T angle = acosf(constrain_float(v1_norm.dot(v2_norm), -1.0f, 1.0f));

    // Set quaternion from axis-angle
    T half_angle = angle * 0.5f;
    T sin_half = sinf(half_angle);

    w = cosf(half_angle);
    x = axis.x * sin_half;
    y = axis.y * sin_half;
    z = axis.z * sin_half;
}

// Check if valid unit quaternion
template <typename T>
bool Quaternion<T>::is_valid(void) const
{
    // Check for NaN values
    if (is_nan()) {
        return false;
    }

    // Check magnitude is close to 1.0
    T mag = magnitude();
    return (mag > 0.99f && mag < 1.01f);
}

// Get angle to another quaternion
template <typename T>
T Quaternion<T>::angle_to(const Quaternion<T> &q) const
{
    T dot = dot(q);
    return acosf(constrain_float(dot, -1.0f, 1.0f)) * 2.0f;
}

// Get shortest angle to another quaternion
template <typename T>
T Quaternion<T>::shortest_angle_to(const Quaternion<T> &q) const
{
    T dot = dot(q);
    if (dot < 0.0f) {
        return acosf(constrain_float(-dot, -1.0f, 1.0f)) * 2.0f;
    }
    return acosf(constrain_float(dot, -1.0f, 1.0f)) * 2.0f;
}

// Normalize angle to [0, 2*PI)
float normalize_angle(float angle)
{
    return fmodf(angle + 2.0f * M_PI, 2.0f * M_PI);
}

// Normalize angle to [-PI, PI)
float normalize_half_angle(float angle)
{
    angle = fmodf(angle + M_PI, 2.0f * M_PI);
    if (angle < 0.0f) {
        angle += 2.0f * M_PI;
    }
    if (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    return angle;
}

// Wrap angle to [0, 2*PI)
float wrap_2pi(float angle)
{
    while (angle > 2.0f * M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < 0.0f) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

// Wrap angle to [-PI, PI)
float wrap_pi(float angle)
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

// Fast math approximations
float fast_sqrt(float x)
{
    return sqrtf(x);
}

float fast_inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

// Explicit template instantiations
template void Quaternion<float>::to_matrix(Matrix3<float> &m) const;
template void Quaternion<float>::from_matrix(const Matrix3<float> &m);
template void Quaternion<float>::from_euler(const Vector3<float> &euler);
template void Quaternion<float>::to_euler(float *roll, float *pitch, float *yaw) const;
template Vector3<float> Quaternion<float>::get_euler(void) const;
template Vector3<float> Quaternion<float>::rotate_vector(const Vector3<float> &v) const;
template Vector3<float> Quaternion<float>::inverse_rotate_vector(const Vector3<float> &v) const;
template void Quaternion<float>::normalize_if_needed(void);
template Quaternion<float> Quaternion<float>::inverse(void) const;
template Quaternion<float> Quaternion<float>::slerp(const Quaternion<float> &q, const T t) const;
template Vector3<float> Quaternion<float>::get_gyro(const Quaternion<float> &dq) const;
template void Quaternion<float>::integrate_gyro(const Vector3<float> &gyro, const T dt);
template Quaternion<float> Quaternion<float>::get_derivative(const Vector3<float> &gyro) const;
template void Quaternion<float>::set_from_two_vectors(const Vector3<float> &v1, const Vector3<float> &v2);
template bool Quaternion<float>::is_valid(void) const;
template float Quaternion<float>::angle_to(const Quaternion<float> &q) const;
template float Quaternion<float>::shortest_angle_to(const Quaternion<float> &q) const;