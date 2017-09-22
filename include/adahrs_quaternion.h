/*
 * adahrs_quaternion.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_QUATERNION_H_
#define ADAHRS_QUATERNION_H_

#include "cmsis_device.h"

// ----------------------------------------------------------------------------

class Quaternion
{
public:

    Quaternion();
    explicit Quaternion(float a, float b, float c, float d);
    Quaternion(const Quaternion& src);

    float a() const;
    float b() const;
    float c() const;
    float d() const;
    
    // operators
    Quaternion& operator =(const Quaternion& src);
    Quaternion& operator *(const Quaternion& other);
    Quaternion& operator +(const Quaternion& other);
    Quaternion& operator -(const Quaternion& other);
    Quaternion& operator *(float scalar);

    // functions
    void norm();
    void conj();
    
private:
    float _a,_b,_c,_d;
};

// ----------------------------------------------------------------------------
#endif /* ADAHRS_QUATERNION_H_ */
