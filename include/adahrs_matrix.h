/*
 * adahrs_matrix.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_MATRIX_H_
#define ADAHRS_MATRIX_H_

// ----------------------------------------------------------------------------
#define	MATRIX_MAX_ROWS		4
#define	MATRIX_MAX_COLUMNS	4

typedef struct _fMatrix {
    int rows;
    int columns;
    float data[MATRIX_MAX_ROWS][MATRIX_MAX_COLUMNS];
} fMatrix;

// Matrix operations
//int mat_add( fMatrix* src1, fMatrix* src2, fMatrix* dest );
//int mat_mult( fMatrix* src1, fMatrix* src2, fMatrix* dest );
//int mat_scalar_mult( float scalar, fMatrix* src, fMatrix* dest );
//int mat_determinant( fMatrix* src, float* det );
//int mat_transpose( fMatrix* src, fMatrix* dest );
//int mat_create_identity( fMatrix* dest, int rows, int columns );
//int mat_zero( fMatrix* dest, int rows, int columns );
//int mat_copy( fMatrix* src, fMatrix* dest );

//void mat_print( fMatrix* matrix );

// ----------------------------------------------------------------------------
#endif /* ADAHRS_MATRIX_H_ */
