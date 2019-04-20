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

#include "collision.h"
#include <debug.h>
#include "constants.h"

namespace rhoban_ssl
{
std::pair<bool, double> collisionTime(double radius_1, const Movement& movement_1, double radius_2,
                                      const Movement& movement_2, double radius_error, double time)
{
  return collisionTime(radius_1, movement_1.linearPosition(time), movement_1.linearVelocity(time), radius_2,
                       movement_2.linearPosition(time), movement_2.linearVelocity(time), radius_error);
}

std::pair<bool, double> collisionTime(double radius_A, const rhoban_geometry::Point& A, const Vector2d& V_A,
                                      double radius_B, const rhoban_geometry::Point& B, const Vector2d& V_B,
                                      double radius_error)
{
  std::pair<bool, double> result(false, 0.0);

  Vector2d V = V_B - V_A;
  Vector2d T = B - A;
  double full_radius = radius_error + radius_A + radius_B;

  double a = V.normSquare();
  double b = 2 * scalarProduct(V, T);
  double c = T.normSquare() - full_radius * full_radius;

  if (a < EPSILON_VELOCITY)
  {
    if (c < EPSILON_DISTANCE)
    {
      return std::pair<bool, double>(true, 0.0);
    }
  }
  else
  {
    double discriminant = b * b - 4 * a * c;
    if (discriminant >= 0)
    {
      double square_discriminant = std::sqrt(discriminant);
      double t_p = (-b + square_discriminant) / (2.0 * a);
      double t_m = (-b - square_discriminant) / (2.0 * a);
      if (0 < t_m)
      {
        result.first = true;
        result.second = t_m;
      }
      else if (0 <= t_p)
      {
        result.first = true;
        result.second = 0.0;
      }
    }
  }
  return result;
}

}  // namespace rhoban_ssl
