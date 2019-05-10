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
double MovementWithNoPrediction::lastTime() const
{
  return samples_.time(0);
}

void MovementWithNoPrediction::print(std::ostream& stream) const
{
  stream << samples_;
}

void MovementWithNoPrediction::setSample(const MovementSample& samples)
{
  // TODO
  // assert( samples.is_valid() );
  samples_ = samples;
}

const MovementSample& MovementWithNoPrediction::getSample() const
{
  return samples_;
}

rhoban_geometry::Point MovementWithNoPrediction::linearPosition(double time) const
{
  return samples_.linearPosition(0);
}

ContinuousAngle MovementWithNoPrediction::angularPosition(double time) const
{
  return samples_.angularPosition(0);
}

Vector2d MovementWithNoPrediction::linearVelocity(double time) const
{
  return samples_.linearVelocity(0);
}

ContinuousAngle MovementWithNoPrediction::angularVelocity(double time) const
{
  return samples_.angularVelocity(0);
}

Vector2d MovementWithNoPrediction::linearAcceleration(double time) const
{
  return samples_.linearAcceleration(0);
}

ContinuousAngle MovementWithNoPrediction::angularAcceleration(double time) const
{
  return samples_.angularAcceleration(0);
}

Movement* MovementWithNoPrediction::clone() const
{
  MovementWithNoPrediction* mov = new MovementWithNoPrediction();
  *mov = *this;
  return mov;
}

MovementWithNoPrediction::~MovementWithNoPrediction()
{
}

}  // namespace rhoban_ssl
