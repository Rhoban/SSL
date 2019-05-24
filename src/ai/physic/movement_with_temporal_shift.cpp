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

#include "movement_with_temporal_shift.h"
#include "data.h"

namespace rhoban_ssl
{


Movement* MovementWithTemporalShift::clone() const
{
  return new MovementWithTemporalShift(movement_->clone());
}
const Movement* MovementWithTemporalShift::getOriginalMovement() const
{
  return movement_;
}

void MovementWithTemporalShift::setSample(const MovementSample& samples)
{
  movement_->setSample(samples);
}
const MovementSample& MovementWithTemporalShift::getSample() const
{
  return movement_->getSample();
}

double MovementWithTemporalShift::temporalShift() const
{
  return GlobalDataSingleThread::singleton_.ai_data_.time_shift_with_vision;
}

double MovementWithTemporalShift::lastTime() const
{
  return movement_->lastTime() + temporalShift();
};

rhoban_geometry::Point MovementWithTemporalShift::linearPosition(double time) const
{
  return movement_->linearPosition(time - temporalShift());
}
ContinuousAngle MovementWithTemporalShift::angularPosition(double time) const
{
  return movement_->angularPosition(time - temporalShift());
}

Vector2d MovementWithTemporalShift::linearVelocity(double time) const
{
  return movement_->linearVelocity(time - temporalShift());
}
ContinuousAngle MovementWithTemporalShift::angularVelocity(double time) const
{
  return movement_->angularVelocity(time - temporalShift());
}

Vector2d MovementWithTemporalShift::linearAcceleration(double time) const
{
  return movement_->linearAcceleration(time - temporalShift());
}
ContinuousAngle MovementWithTemporalShift::angularAcceleration(double time) const
{
  return movement_->angularAcceleration(time - temporalShift());
}

void MovementWithTemporalShift::print(std::ostream& stream) const
{
  return movement_->print(stream);
}

MovementWithTemporalShift::~MovementWithTemporalShift()
{
  delete movement_;
}

}  // namespace rhoban_ssl
