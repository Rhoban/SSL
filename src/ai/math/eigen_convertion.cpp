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

#include "eigen_convertion.h"

rhoban_geometry::Point eigen2point(const Vector2d& v)
{
  return rhoban_geometry::Point(v[0], v[1]);
}

Vector2d eigen2vector(const Vector2d& v)
{
  return Vector2d(v[0], v[1]);
}

Vector2d point2eigen(const rhoban_geometry::Point& v)
{
  return Vector2d(v.getX(), v.getY());
}

Vector2d vector2eigen(const Vector2d& v)
{
  return Vector2d(v.getX(), v.getY());
}
