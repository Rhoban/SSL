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
Point centerOfConeIncircle(const rhoban_geometry::Point& cone_vertex, const rhoban_geometry::Point& cone_base_A,
                           const rhoban_geometry::Point& cone_base_B, double circle_radius)
{
  Vector2d u = normalized(cone_base_A - cone_vertex);
  Vector2d v = normalized(cone_base_B - cone_vertex);
  double u_vec_v = vectorialProduct(u, v);
  assert(u_vec_v != 0.0);
  double signe = (u_vec_v >= 0) ? 1 : -1;
  return vector2point(((u + v) * (signe * circle_radius) + v * vectorialProduct(u, cone_vertex) +
                       u * vectorialProduct(cone_vertex, v)) /
                      u_vec_v);
}

std::vector<rhoban_geometry::Segment> tangentOfTwoCircle(const rhoban_geometry::Circle& circle_A,
                                                         const rhoban_geometry::Circle& circle_B)
{
  std::vector<rhoban_geometry::Segment> result(4);

  const rhoban_geometry::Point& A = circle_A.getCenter();
  double r_a = circle_A.getRadius();
  const rhoban_geometry::Point& B = circle_B.getCenter();
  double r_b = circle_B.getRadius();
  double norm_AB = norm(B - A);
  Vector2d x = normalized(B - A);
  Vector2d y(-x.getY(), x.getX());

  auto fct = [&](double epsilon_1, double epsilon_2) {
    double cos_theta = (r_a - epsilon_1 * r_b) / norm_AB;
    double sin_theta = epsilon_2 * std::sqrt(1 - cos_theta * cos_theta);
    Vector2d u = x * cos_theta + y * sin_theta;
    rhoban_geometry::Point M = A + u * r_a;
    rhoban_geometry::Point N = B + u * (epsilon_1 * r_b);
    return rhoban_geometry::Segment(M, N);
  };

  result[0] = fct(1.0, 1.0);
  result[1] = fct(1.0, -1.0);
  result[2] = fct(-1.0, 1.0);
  result[3] = fct(-1.0, -1.0);

  return result;
}

}  // namespace rhoban_geometry
