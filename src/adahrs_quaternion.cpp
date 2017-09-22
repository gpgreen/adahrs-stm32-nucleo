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
    
Quaternion& Quaternion::operator *=(const Quaternion& rhs)
{
    float a = _a;
    float b = _b;
    float c = _c;
    float d = _d;
    
    _a = a * rhs._a - b * rhs._b - c * rhs._c - d * rhs._d;
    _b = a * rhs._b + b * rhs._a + c * rhs._d - d * rhs._c;
    _c = a * rhs._c - b * rhs._d + c * rhs._a + d * rhs._b;
    _d = a * rhs._d + b * rhs._c - c * rhs._b + d * rhs._a;

    return *this;
}

Quaternion& Quaternion::operator +=(const Quaternion& rhs)
{
    _a += rhs._a;
    _b += rhs._b;
    _c += rhs._c;
    _d += rhs._d;

    return *this;
}

Quaternion& Quaternion::operator -=(const Quaternion& rhs)
{
    _a -= rhs._a;
    _b -= rhs._b;
    _c -= rhs._c;
    _d -= rhs._d;

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

void Quaternion::normalize()
{
    float norm = _a * _a + _b * _b + _c * _c + _d * _d;
    norm = sqrt(norm);
    _a /= norm;
    _b /= norm;
    _c /= norm;
    _d /= norm;
}

void Quaternion::conjugation()
{
    _b = -_b;
    _c = -_c;
    _d = -_d;
}

// ----------------------------------------------------------------------------

