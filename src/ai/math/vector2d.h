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

#ifndef __TOOLS__VECTOR_H__
#define __TOOLS__VECTOR_H__

#include <math/ContinuousAngle.h>
#include <rhoban_geometry/point.h>

// This is a hack in order to prepare a future refactoring that introduces a
// Vector class.


class Vector2d {
    public:
    double vec[2];
    
    Vector2d( double x, double y);
    Vector2d();
    Vector2d(const rhoban_geometry::Point& point);

    double getX() const;
    double getY() const;
    double operator[](unsigned int i) const;
    double & operator[](unsigned int i);

    double norm() const;
    double norm_square() const;

    Vector2d operator-() const;
    const Vector2d & operator+() const;
    Vector2d operator+( const Vector2d& v ) const;
    Vector2d & operator+=( const Vector2d& v );
    Vector2d operator-( const Vector2d& v ) const;
    Vector2d & operator-=( const Vector2d& v );
    Vector2d & operator*=( double alpha );
    Vector2d operator*( double alpha ) const;
    Vector2d & operator/=( double alpha );
    Vector2d operator/( double alpha ) const;

    Vector2d & operator=( const Vector2d & v );

    Vector2d perpendicular();
};

rhoban_geometry::Point vector2point( const Vector2d & v );
Vector2d point2vector( const rhoban_geometry::Point & p );

bool operator==(const Vector2d & v1, const Vector2d & v2);
bool operator!=(const Vector2d & v1, const Vector2d & v2);

rhoban_geometry::Point operator+(const rhoban_geometry::Point & p, const Vector2d & v);
rhoban_geometry::Point operator+(const Vector2d & v, const rhoban_geometry::Point & p);
rhoban_geometry::Point operator-(const rhoban_geometry::Point & p, const Vector2d & v);

double vectorial_product( const Vector2d & v1, const Vector2d & v2 );
double scalar_product( const Vector2d & v1, const Vector2d & v2 );


double norm( const Vector2d & v1 );
double norm_2( const Vector2d & v1 );
double norm_square( const Vector2d & v1 );
Vector2d normalized( const Vector2d & v1 );

ContinuousAngle vector2angle( Vector2d direction );

Vector2d operator*(double alpha, const Vector2d & v);

std::ostream& operator<<(std::ostream& out, const Vector2d& v);

#endif
