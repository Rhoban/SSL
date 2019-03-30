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

#include "movement_with_no_prediction.h"
#include <debug.h>

namespace rhoban_ssl
{
double Movement_with_no_prediction::lastTime() const
{
  return samples.time(0);
}

void Movement_with_no_prediction::print(std::ostream& stream) const
{
  stream << samples;
}

void Movement_with_no_prediction::setSample(const MovementSample& samples)
{
  // TODO
  // assert( samples.is_valid() );
  this->samples = samples;
}

const MovementSample& Movement_with_no_prediction::getSample() const
{
  return samples;
}

rhoban_geometry::Point Movement_with_no_prediction::linearPosition(double time) const
{
  return samples.linear_position(0);
}

ContinuousAngle Movement_with_no_prediction::angularPosition(double time) const
{
  return samples.angular_position(0);
}

Vector2d Movement_with_no_prediction::linearVelocity(double time) const
{
  return samples.linear_velocity(0);
}

ContinuousAngle Movement_with_no_prediction::angularVelocity(double time) const
{
  return samples.angular_velocity(0);
}

Vector2d Movement_with_no_prediction::linearAcceleration(double time) const
{
  return samples.linear_acceleration(0);
}

ContinuousAngle Movement_with_no_prediction::angularAcceleration(double time) const
{
  return samples.angular_acceleration(0);
}

Movement* Movement_with_no_prediction::clone() const
{
  Movement_with_no_prediction* mov = new Movement_with_no_prediction();
  *mov = *this;
  return mov;
}

Movement_with_no_prediction::~Movement_with_no_prediction()
{
}

}  // namespace rhoban_ssl
