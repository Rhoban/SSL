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
  std::vector<rhoban_geometry::Point> intersections;

  assert(point_line_1 != point_line_2);
  if (point_line_1 == point_line_2)
    return intersections;

  const Point& a = point_line_1;
  const Point& b = point_line_2;

  Vector2d a_b = b - a;
  double dx = a_b.getX();
  double dy = a_b.getY();

  double D = Point::perpDotProduct(a, b);

  double discrim = circle.getRadius() * circle.getRadius() * a_b.normSquare() - D * D;

  double sign = (dy < 0) ? -1 : 1;

  if (discrim < 0)
    return intersections;

  if (discrim == 0.0000)
  {
    Point tangent(D * dy / a_b.normSquare(),
                  (-D) * dx / a_b.normSquare());
    intersections.push_back(tangent);
  }

  if (discrim > 0)
  {
    Point point_1((D * dy + sign * dx * sqrt(discrim)) / a_b.normSquare(),
                  ((-D) * dx + std::abs(dy) * sqrt(discrim)) / a_b.normSquare());
    Point point_2((D * dy - sign * dx * sqrt(discrim)) / a_b.normSquare(),
                  ((-D) * dx - std::abs(dy) * sqrt(discrim)) / a_b.normSquare());
    intersections.push_back(point_1);
    intersections.push_back(point_2);
  }

  return intersections;
}

}  // namespace rhoban_geometry
