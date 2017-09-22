/*
 * adahrs_quaternion.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: ggreen
 */

#include <math.h>

#include "adahrs_quaternion.h"

// ----------------------------------------------------------------------------

Quaternion::Quaternion()
    : _a(0), _b(0), _c(0), _d(0)
{
    // does nothing else
}

Quaternion::Quaternion(float a, float b, float c, float d)
    : _a(a), _b(b), _c(c), _d(d)
{
    // does nothing else
}

Quaternion::Quaternion(const Quaternion& src)
{
    if (&src == this)
        return;
    _a = src._a;
    _b = src._b;
    _c = src._c;
    _d = src._d;
}

Quaternion& Quaternion::operator =(const Quaternion& src)
{
    if (&src == this)
        return *this;
    _a = src._a;
    _b = src._b;
    _c = src._c;
    _d = src._d;
    return *this;
}
    
Quaternion& Quaternion::operator *(const Quaternion& other)
{
    float a = _a;
    float b = _b;
    float c = _c;
    float d = _d;
    
    _a = a * other._a - b * other._b - c * other._c - d * other._d;
    _b = a * other._b + b * other._a + c * other._d - d * other._c;
    _c = a * other._c - b * other._d + c * other._a + d * other._b;
    _d = a * other._d + b * other._c - c * other._b + d * other._a;

    return *this;
}

Quaternion& Quaternion::operator +(const Quaternion& other)
{
    _a += other._a;
    _b += other._b;
    _c += other._c;
    _d += other._d;

    return *this;
}

Quaternion& Quaternion::operator -(const Quaternion& other)
{
    _a -= other._a;
    _b -= other._b;
    _c -= other._c;
    _d -= other._d;

    return *this;
}

Quaternion& Quaternion::operator *(float scalar)
{
    _a *= scalar;
    _b *= scalar;
    _c *= scalar;
    _d *= scalar;

    return *this;
}

void Quaternion::norm()
{
    float norm = _a * _a + _b * _b + _c * _c + _d * _d;
    norm = sqrt(norm);
    _a /= norm;
    _b /= norm;
    _c /= norm;
    _d /= norm;
}

void Quaternion::conj()
{
    _b = -_b;
    _c = -_c;
    _d = -_d;
}

// ----------------------------------------------------------------------------

