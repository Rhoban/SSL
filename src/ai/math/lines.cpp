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

#include "tangents.h"
#include "vector2d.h"
#include <debug.h>
namespace rhoban_geometry
{
double distance_from_point_to_line(const rhoban_geometry::Point& point, const rhoban_geometry::Point& point_line_1,
                                   const rhoban_geometry::Point& point_line_2)
{
  assert(norm_square(point_line_1 - point_line_2) != 0);

  Vector2d p1p2 = point_line_2 - point_line_1;
  Vector2d p1p = point - point_line_1;
  Vector2d u = p1p2 / p1p2.norm();
  return std::fabs(vectorial_product(u, p1p));
}

}  // namespace rhoban_geometry
