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

#pragma once

#include <rhoban_geometry/point.h>
#include <math/continuous_angle.h>
#include <math/vector2d.h>
#include <math/circular_vector.h>
namespace rhoban_ssl
{
struct PositionSample
{
  double time;
  rhoban_geometry::Point linear_position;
  ContinuousAngle angular_position;

  PositionSample();
  PositionSample(double time, const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position);
};

struct MovementSample : public CircularVector<PositionSample>
{
  CircularVector<double> dts;

  MovementSample(unsigned int, double default_dt = 1.0 / 60.0);
  MovementSample();

  double time(unsigned int i = 0) const;
  double dt(unsigned int i = 0) const;

  rhoban_geometry::Point linearPosition(unsigned int i = 0) const;
  ContinuousAngle angularPosition(unsigned int i = 0) const;

  Vector2d linearVelocity(unsigned int i = 0) const;
  ContinuousAngle angularVelocity(unsigned int i = 0) const;

  Vector2d linearAcceleration(unsigned int i = 0) const;
  ContinuousAngle angularAcceleration(unsigned int i = 0) const;

  bool isValid() const;
  void insert(const PositionSample& sample);
};

}  // namespace rhoban_ssl

std::ostream& operator<<(std::ostream& stream, const rhoban_ssl::PositionSample& pos);

std::ostream& operator<<(std::ostream& stream, const rhoban_ssl::MovementSample& mov);
