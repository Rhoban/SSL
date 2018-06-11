/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __TOOLS__MATRIX2D_H__
#define __TOOLS__MATRIX2D_H__

#include "vector2d.h"
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

template <typename MATRIX_CLASS>
Matrix2d inverse( const MATRIX_CLASS & m ){
    return Matrix2d( m ).inverse();
}

#endif
