/*
 * adahrs_matrix.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_MATRIX_H_
#define ADAHRS_MATRIX_H_

#include "cmsis_device.h"

// ----------------------------------------------------------------------------
#define	MATRIX_MAX_ROWS		4
#define	MATRIX_MAX_COLUMNS	4

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class Matrix
{
public:
    // constructor(s)
    Matrix();
    explicit Matrix(uint8_t rows, uint8_t cols, bool identity=false);
    Matrix(const Matrix& src);

    // get value
    float get(uint8_t row, uint8_t col) const;
    void set(uint8_t row, uint8_t col, float val);
    
    // assignment operator
    Matrix& operator =(const Matrix& m);

    // matrix operators
    Matrix& operator +(const Matrix& m);
    Matrix& operator *(const Matrix& m);
    Matrix& operator *(float scalar);

    // functions
    float determinant() const;
    Matrix& transpose(const Matrix& src);
    void zero();

    void print() const;
    
private:
    
    float _data[MATRIX_MAX_ROWS][MATRIX_MAX_COLUMNS];
    uint8_t _rows;
    uint8_t _cols;
};

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
#endif /* ADAHRS_MATRIX_H_ */
