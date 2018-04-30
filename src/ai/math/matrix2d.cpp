#include "matrix2d.h"

Matrix2d::Matrix2d( double a, double b, double c , double d):
    Boost_Matrix2d(2,2)
{
    (*this)(0,0) = a;
    (*this)(0,1) = b;
    (*this)(1,0) = c;
    (*this)(1,1) = d;
}

Matrix2d::Matrix2d():
    Matrix2d(0.0, 0.0, 0.0, 0.0)
{}

double Matrix2d::det() const {
    return (*this)(0,0)*(*this)(1,1) - (*this)(0,1)*(*this)(1,0);
}

Matrix2d Matrix2d::inverse() const {
    double det = this->det();
    return Matrix2d(
        (*this)(1,1)/det, -(*this)(0,1)/det, 
        -(*this)(1,0)/det, (*this)(0,0)/det
    );
}
