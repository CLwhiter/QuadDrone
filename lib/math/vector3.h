/* Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0 */

#ifndef APP_MATH_VECTOR3_H_
#define APP_MATH_VECTOR3_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include "rotations.h"

#ifdef __cplusplus
extern "C" {
#endif

template <typename T>
class Vector3
{
public:
    static_assert(std::is_floating_point<T>::value, "Vector3 only supports floating-point types");
    T x, y, z;

    // Constructors
    Vector3<T>() {
        x = y = z = 0;
    }

    // Parameterized constructor
    Vector3<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0) {
    }

    void operator ()(const T x0, const T y0, const T z0)
    {
        x = x0; y = y0; z = z0;
    }

    // Rotation
    void rotate(enum Rotation rotation);

    // Get length squared
    T length_squared() const
    {
        return (T)(*this * *this);
    }

    // Get length
    float length(void) const;

    // Normalize this vector
    void normalize()
    {
        float len = length();
        if (len > 0.0f) {
            *this /= len;
        }
    }

    // Set to zero vector
    void zero()
    {
        x = y = z = 0;
    }

    // Return a normalized copy
    Vector3<T> normalized() const
    {
        float len = length();
        if (len > 0.0f) {
            return *this / len;
        }
        return Vector3<T>(0, 0, 0);  // Return zero vector
    }

    // Get angle between two vectors
    float angle(const Vector3<T> &v2) const;

    void get_rollpitch(Vector3<T> &angle);

    void get_yaw(Vector3<T> &angle);

    // Check if any element value is NAN
    bool is_nan(void) const;

/*************************Operator overloads*********************************/
    // Dot product
    T operator *(const Vector3<T> &v) const;

    // Cross product
    Vector3<T> operator %(const Vector3<T> &v) const;

    // Scalar multiplication
    Vector3<T> operator *(const T num) const;
    Vector3<T> &operator *=(const T num);

    // Scalar division
    Vector3<T> operator /(const T num) const;
    Vector3<T> &operator /=(const T num);

    // Unary negation
    Vector3<T> operator -(void) const;

    // Addition
    Vector3<T> operator +(const Vector3<T> &v) const;
    Vector3<T> &operator +=(const Vector3<T> &v);

    // Subtraction
    Vector3<T> operator -(const Vector3<T> &v) const;
    Vector3<T> &operator -=(const Vector3<T> &v);

    // Equality comparison
    bool operator ==(const Vector3<T> &v) const;

    // Inequality comparison
    bool operator !=(const Vector3<T> &v) const;

/*******************************************************************/

};

// Type aliases
typedef Vector3<float>                  Vector3f;
typedef Vector3<double>                 Vector3d;

#ifdef __cplusplus
}
#endif

#endif /* APP_MATH_VECTOR3_H_ */