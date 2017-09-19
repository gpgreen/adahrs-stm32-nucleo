/*
 * adahrs_quaternion.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_QUATERNION_H_
#define ADAHRS_QUATERNION_H_

// ----------------------------------------------------------------------------
typedef struct _quat {
	 float a,b,c,d;
} quat;

//int quat_mult( quat* src1, quat* src2, quat* dest );
//int quat_conj( quat* src, quat* dest );
//int quat_norm( quat* src );
//int quat_add( quat* src1, quat* src2, quat* dest );
//int quat_subtract( quat* src1, quat*src2, quat* dest );
//int quat_scalar_mult( quat* qsrc, float scalar, quat* dest );

// ----------------------------------------------------------------------------
#endif /* ADAHRS_QUATERNION_H_ */
