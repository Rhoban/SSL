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

Matrix2d::Matrix2d(double a, double b, double c, double d) : mat({ Vector2d(a, b), Vector2d(c, d) })
{
}

Matrix2d::Matrix2d() : Matrix2d(0.0, 0.0, 0.0, 0.0)
{
}

double Matrix2d::det() const
{
  return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
}

Matrix2d Matrix2d::inverse() const
{
  double det = this->det();
  return Matrix2d((*this)(1, 1) / det, -(*this)(0, 1) / det, -(*this)(1, 0) / det, (*this)(0, 0) / det);
}

Vector2d& Matrix2d::operator[](unsigned int i)
{
  assert(i < 2);
  return mat[i];
}
const Vector2d& Matrix2d::operator[](unsigned int i) const
{
  assert(0 <= i and i < 2);
  return mat[i];
}

double& Matrix2d::operator()(unsigned int i, unsigned int j)
{
  return mat[i][j];
}

double Matrix2d::operator()(unsigned int i, unsigned int j) const
{
  return mat[i][j];
}

Matrix2d Matrix2d::operator*(const Matrix2d& m2) const
{
  return Matrix2d(mat[0][0] * m2[0][0] + mat[0][1] * m2[1][0], mat[0][0] * m2[0][1] + mat[0][1] * m2[1][1],
                  mat[1][0] * m2[0][0] + mat[1][1] * m2[1][0], mat[1][0] * m2[0][1] + mat[1][1] * m2[1][1]);
}

Matrix2d Matrix2d::operator*(double alpha) const
{
  return Matrix2d(mat[0][0] * alpha, mat[0][1] * alpha, mat[1][0] * alpha, mat[1][1] * alpha);
}

Vector2d Matrix2d::operator*(const Vector2d& v) const
{
  return Vector2d(mat[0][0] * v[0] + mat[0][1] * v[1], mat[1][0] * v[0] + mat[1][1] * v[1]);
}

Matrix2d Matrix2d::identity()
{
  return Matrix2d(1.0, 0.0, 0.0, 1.0);
}

Matrix2d Matrix2d::null()
{
  return Matrix2d(0.0, 0.0, 0.0, 0.0);
}

std::ostream& operator<<(std::ostream& out, const Matrix2d& v)
{
  out << "[" << v[0] << ", " << v[1] << "]";
  return out;
}

Matrix2d operator*(double alpha, const Matrix2d& m)
{
  return m * alpha;
}

const Matrix2d& Matrix2d::operator+() const
{
  return *this;
}

Matrix2d Matrix2d::operator+(const Matrix2d& m2) const
{
  return Matrix2d(mat[0][0] + m2.mat[0][0], mat[0][1] + m2.mat[0][1], mat[1][0] + m2.mat[1][0],
                  mat[1][1] + m2.mat[1][1]);
}

Matrix2d Matrix2d::operator-() const
{
  return Matrix2d(-mat[0][0], -mat[0][1], -mat[1][0], -mat[1][1]);
}

Matrix2d Matrix2d::operator-(const Matrix2d& m2) const
{
  return Matrix2d(mat[0][0] - m2.mat[0][0], mat[0][1] - m2.mat[0][1], mat[1][0] - m2.mat[1][0],
                  mat[1][1] - m2.mat[1][1]);
}
