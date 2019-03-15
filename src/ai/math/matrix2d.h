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

class Matrix2d
{
public:
  Vector2d mat[2];

  Matrix2d(double a, double b, double c, double d);
  Matrix2d();

  double det() const;
  Matrix2d inverse() const;

  Vector2d& operator[](unsigned int i);
  const Vector2d& operator[](unsigned int i) const;

  double& operator()(unsigned int i, unsigned int j);
  double operator()(unsigned int i, unsigned int j) const;

  Matrix2d operator*(const Matrix2d& m2) const;
  Matrix2d operator*(double alpha) const;
  Vector2d operator*(const Vector2d& v) const;

  const Matrix2d& operator+() const;
  Matrix2d operator+(const Matrix2d& m2) const;
  Matrix2d operator-() const;
  Matrix2d operator-(const Matrix2d& m2) const;

  static Matrix2d identity();
  static Matrix2d null();
};

std::ostream& operator<<(std::ostream& out, const Matrix2d& v);
Matrix2d operator*(double alpha, const Matrix2d& m);

#endif
