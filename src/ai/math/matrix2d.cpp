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
