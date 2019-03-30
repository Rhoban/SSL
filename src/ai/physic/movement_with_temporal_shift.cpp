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

namespace rhoban_ssl
{
Movement_with_temporal_shift::Movement_with_temporal_shift(Movement* movement, std::function<double()> temporal_shift)
  : movement(movement), temporal_shift(temporal_shift)
{
}

Movement* Movement_with_temporal_shift::clone() const
{
  return new Movement_with_temporal_shift(movement->clone(), temporal_shift);
}
const Movement* Movement_with_temporal_shift::get_original_movement() const
{
  return movement;
}

void Movement_with_temporal_shift::setSample(const MovementSample& samples)
{
  movement->setSample(samples);
}
const MovementSample& Movement_with_temporal_shift::getSample() const
{
  return movement->getSample();
}

double Movement_with_temporal_shift::lastTime() const
{
  return movement->lastTime() + temporal_shift();
};

rhoban_geometry::Point Movement_with_temporal_shift::linearPosition(double time) const
{
  return movement->linearPosition(time - temporal_shift());
}
ContinuousAngle Movement_with_temporal_shift::angularPosition(double time) const
{
  return movement->angularPosition(time - temporal_shift());
}

Vector2d Movement_with_temporal_shift::linearVelocity(double time) const
{
  return movement->linearVelocity(time - temporal_shift());
}
ContinuousAngle Movement_with_temporal_shift::angularVelocity(double time) const
{
  return movement->angularVelocity(time - temporal_shift());
}

Vector2d Movement_with_temporal_shift::linearAcceleration(double time) const
{
  return movement->linearAcceleration(time - temporal_shift());
}
ContinuousAngle Movement_with_temporal_shift::angularAcceleration(double time) const
{
  return movement->angularAcceleration(time - temporal_shift());
}

void Movement_with_temporal_shift::print(std::ostream& stream) const
{
  return movement->print(stream);
}

Movement_with_temporal_shift::~Movement_with_temporal_shift()
{
  delete movement;
}

}  // namespace rhoban_ssl
