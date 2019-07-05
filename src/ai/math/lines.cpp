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
double distanceFromPointToLine(const rhoban_geometry::Point& point, const rhoban_geometry::Point& point_line_1,
                               const rhoban_geometry::Point& point_line_2)
{
  assert(normSquare(point_line_1 - point_line_2) != 0);

  Vector2d p1p2 = point_line_2 - point_line_1;
  Vector2d p1p = point - point_line_1;
  Vector2d u = p1p2 / p1p2.norm();
  return std::fabs(vectorialProduct(u, p1p));
}

/**
 * @brief Implementation form
 *
 * @ref http://mathworld.wolfram.com/Circle-LineIntersection.html
 */
std::vector<rhoban_geometry::Point> getIntersectionLineWithCircle(const Point& point_line_1, const Point& point_line_2,
                                                                  const Circle& circle)
{
  std::vector<rhoban_geometry::Point> intersections; //correctement initialisé ?

  assert(point_line_1 != point_line_2);
  if (point_line_1 == point_line_2)
    return intersections;

  //equation ax + by = c
  double a = 1;
  double b = 1;
  double c = 0;
  double dx = point_line_2.getX() - point_line_1.getX();
  double dy = point_line_2.getY() - point_line_1.getY();
  if(dx != 0.00) {
    a = dy / dx;
    b = -1;
    c = -(point_line_1.getY() - a * point_line_1.getX());
  }
  else {
    a = 1;
    b = 0;
    c = point_line_1.getX();
  }
  double dr_square = a*a + b*b;

  //equation (x-xm)² + (y - ym)² = r²
  double xm = circle.getCenter().getX();
  double ym = circle.getCenter().getY();
  double r = circle.getRadius();
  double c_sec = c - a*xm - b*ym;


  double discrim = r*r * dr_square - c_sec * c_sec;

  if (discrim < 0)
    return intersections;

  if (discrim == 0.0000)
  {
    Point tangent(c_sec * a / dr_square + xm,
                  (-c_sec) * b /dr_square +ym);
    intersections.push_back(tangent);
  }

  if (discrim > 0)
  {
    Point point_1((a * c_sec + b * sqrt(discrim)) /dr_square + circle.getCenter().getX(),
                  (b * c_sec - a * sqrt(discrim)) /dr_square + circle.getCenter().getY());
    Point point_2((a * c_sec - b * sqrt(discrim)) /dr_square + circle.getCenter().getX(),
                  (b * c_sec + a * sqrt(discrim)) /dr_square + circle.getCenter().getY());
    intersections.push_back(point_1);
    intersections.push_back(point_2);
  }

  return intersections;
}

}  // namespace rhoban_geometry
