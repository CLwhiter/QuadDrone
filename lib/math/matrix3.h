/*
 * Copyright (c) 2025 ANO Open Source Flight Control
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_LIB_MATH_MATRIX3_H_
#define APP_LIB_MATH_MATRIX3_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "math/vector3.h"

// 3x3 matrix with elements of type T
template <typename T>
class Matrix3 {
public:
    static_assert(std::is_floating_point<T>::value, "Matrix3 only supports floating-point types");

    // Matrix columns
    Vector3<T> a, b, c;

    // Default constructor
    Matrix3<T>() {
    }

    // Parameterized constructor
    Matrix3<T>(const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0) : a(a0), b(b0), c(c0) {
    }

    // Parameterized constructor with individual values
    Matrix3<T>(const T ax, const T ay, const T az, const T bx, const T by, const T bz, const T cx, const T cy, const T cz) : a(ax,ay,az), b(bx,by,bz), c(cx,cy,cz) {
    }

    // Set matrix values
    void operator () (const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0)
    {
        a = a0; b = b0; c = c0;
    }

    // Check if matrices are equal
    bool operator == (const Matrix3<T> &m)
    {
        return (a==m.a && b==m.b && c==m.c);
    }

    // Check if matrices are not equal
    bool operator != (const Matrix3<T> &m)
    {
        return (a!=m.a || b!=m.b || c!=m.c);
    }

    // Unary negation
    Matrix3<T> operator - (void) const
    {
        return Matrix3<T>(-a,-b,-c);
    }

    // Addition
    Matrix3<T> operator + (const Matrix3<T> &m) const
    {
        return Matrix3<T>(a+m.a, b+m.b, c+m.c);
    }
    Matrix3<T> &operator += (const Matrix3<T> &m)
    {
        return *this = *this + m;
    }

    // Subtraction
    Matrix3<T> operator - (const Matrix3<T> &m) const
    {
        return Matrix3<T>(a-m.a, b-m.b, c-m.c);
    }
    Matrix3<T> &operator -= (const Matrix3<T> &m)
    {
        return *this = *this - m;
    }

    // Scalar multiplication
    Matrix3<T> operator * (const T num) const
    {
        return Matrix3<T>(a*num, b*num, c*num);
    }
    Matrix3<T> &operator *= (const T num)
    {
        return *this = *this * num;
    }
    Matrix3<T> operator / (const T num) const
    {
        return Matrix3<T>(a/num, b/num, c/num);
    }
    Matrix3<T> &operator /= (const T num)
    {
        return *this = *this / num;
    }

    // Multiply matrix by vector
    Vector3<T> operator *(const Vector3<T> &v) const;

    // Multiply by transposed matrix
    Vector3<T> mul_transpose(const Vector3<T> &v) const;

    // Get x column
    Vector3<T> colx(void) const
    {
        return Vector3<T>(a.x, b.x, c.x);
    }

    // Get y column
    Vector3<T> coly(void) const
    {
        return Vector3<T>(a.y, b.y, c.y);
    }

    // Get z column
    Vector3<T> colz(void) const
    {
        return Vector3<T>(a.z, b.z, c.z);
    }

    // Set z column
    void set_colz( const Vector3<T> v)
    {
        a.z = v.x; b.z = v.y; c.z = v.z;
    }

    // Matrix multiplication
    Matrix3<T> operator *(const Matrix3<T> &m) const;

    Matrix3<T> &operator *= (const Matrix3<T> &m)
    {
        return *this = *this * m;
    }

    // Get transposed matrix
    Matrix3<T> transposed(void) const;

    // Transpose in place
    Matrix3<T> transpose(void)
    {
        return *this = transposed();
    }

    // Set to zero matrix
    void zero(void);

    // Set to identity matrix
    void identity(void) {
        a.x = b.y = c.z = 1;
        a.y = a.z = 0;
        b.x = b.z = 0;
        c.x = c.y = 0;
    }

    // Check if matrix has abnormal values
    bool is_nan(void)
    {
        return a.is_nan() || b.is_nan() || c.is_nan();
    }

    // Convert from Euler angles to rotation matrix
    void from_euler(const Vector3<T> &euler);

    // Convert rotation matrix to Euler angles
    void to_euler(float *roll, float *pitch, float *yaw);

    // Use gyro data to rotate matrix (rotate around z axis)
    void rotate(const Vector3<T> &g);

    // Use gyro data (around X and Y) to rotate matrix
    void rotateXY(const Vector3<T> &g);
};

// Type aliases
typedef Matrix3<float>                  Matrix3f;
typedef Matrix3<double>                 Matrix3d;

#ifdef __cplusplus
}
#endif

#endif /* APP_LIB_MATH_MATRIX3_H_ */