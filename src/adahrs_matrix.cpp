/*
 * adahrs_matrix.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: ggreen
 */

#include "adahrs_matrix.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

Matrix::Matrix()
    : _rows(MATRIX_MAX_ROWS), _cols(MATRIX_MAX_COLUMNS)
{
    zero();
}

Matrix::Matrix(uint8_t rows, uint8_t cols, bool identity)
    : _rows(rows), _cols(cols)
{
    for (int i=0; i<MATRIX_MAX_ROWS; ++i)
    {
        for (int j=0; j<MATRIX_MAX_COLUMNS; ++j)
        {
            if (i == j && identity)
                _data[i][j] = 1.0f;
            else
                _data[i][j] = 0.0f;
        }
    }
}

Matrix::Matrix(const Matrix& src)
{
    if (&src == this)
        return;

    _rows = src._rows;
    _cols = src._cols;
    for (int i=0; i<MATRIX_MAX_ROWS; ++i)
    {
        for (int j=0; j<MATRIX_MAX_COLUMNS; ++j)
        {
            _data[i][j] = src._data[i][j];
        }
    }
}

float Matrix::get(uint8_t row, uint8_t col) const
{
    if (row > _rows || col > _cols)
    {
        UsageFault_Handler();
    }
    return _data[row][col];
}

void Matrix::set(uint8_t row, uint8_t col, float val)
{
    if (row > _rows || col > _cols)
    {
        UsageFault_Handler();
    }
    _data[row][col] = val;
}

Matrix& Matrix::operator =(const Matrix& src)
{
    if (&src == this)
        return *this;

    _rows = src._rows;
    _cols = src._cols;
    
    for (int i=0; i<MATRIX_MAX_ROWS; ++i)
    {
        for (int j=0; j<MATRIX_MAX_COLUMNS; ++j)
        {
            _data[i][j] = src._data[i][j];
        }
    }
    return *this;
}

Matrix& Matrix::operator +=(const Matrix& rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
    {
        UsageFault_Handler();
    }
    
    for (int i=0; i<rhs._rows; ++i)
    {
        for (int j=0; j<rhs._cols; ++j)
        {
            _data[i][j] += rhs._data[i][j];
        }
    }
    return *this;
}

Matrix& Matrix::operator -=(const Matrix& rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
    {
        UsageFault_Handler();
    }

    for (int i=0; i<rhs._rows; ++i)
    {
        for (int j=0; j<rhs._cols; ++j)
        {
            _data[i][j] -= rhs._data[i][j];
        }
    }
    return *this;
}

Matrix& Matrix::operator *= (const Matrix& rhs)
{
    if (_cols != rhs._rows)
    {
        UsageFault_Handler();
    }
    _cols = rhs._cols;
    
    for (int i=0; i<_rows; ++i)
    {
        for (int j=0; j<rhs._cols; ++j)
        {
            float dot_product = 0;
            for (int k=0; k<_cols; ++k)
            {
                dot_product += _data[i][k] * rhs._data[k][j];
            }
            _data[i][j] = dot_product;
        }
    }
    return *this;
}

Matrix& Matrix::operator *(float scalar)
{
    for (int i=0; i<_rows; ++i)
    {
        for (int j=0; j<_cols; ++j)
        {
            _data[i][j] *= scalar;
        }
    }
    return *this;
}

float Matrix::determinant() const
{
    // TODO:
    return 0.0f;
}

Matrix& Matrix::transpose(const Matrix& src)
{
    _rows = src._rows;
    _cols = src._cols;
    for (int i=0; i<_rows; ++i)
    {
        for (int j=0; j<_cols; ++j)
        {
            _data[j][i] = src._data[i][j];
        }
    }
    return *this;
}

void Matrix::zero()
{
    for (int i=0; i<MATRIX_MAX_ROWS; ++i)
    {
        for (int j=0; j<MATRIX_MAX_COLUMNS; ++j)
        {
            _data[i][j] = 0;
        }
    }
}

// ----------------------------------------------------------------------------

