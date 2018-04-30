#ifndef __TOOLS__MATRIX2D_H__
#define __TOOLS__MATRIX2D_H__

#include "vector.h"
#include <boost/numeric/ublas/matrix.hpp>

typedef boost::numeric::ublas::matrix<double> Boost_Matrix2d;

class Matrix2d : public Boost_Matrix2d {
    public:
    Matrix2d( double a, double b, double c , double d);
    Matrix2d();

    template <typename MATRIX_CLASS>
    Matrix2d( const MATRIX_CLASS & m ):
        Boost_Matrix2d(m)
    { }

    double det() const;
    Matrix2d inverse() const;

    Matrix2d operator*(const Matrix2d & m2) const {
        return prec_prod(*this, m2); 
    }
    Vector2d operator*(const Vector2d & m2) const {
        return prec_prod(*this, m2); 
    }
};


#endif
