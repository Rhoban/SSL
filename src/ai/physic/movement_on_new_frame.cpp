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

#include "movement_on_new_frame.h"
#include <debug.h>

namespace rhoban_ssl
{
void Movement_on_new_frame::set_frame(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2)
{
  frame.setFrame(origin, v1, v2);
}

Movement_on_new_frame::Movement_on_new_frame(Movement* movement) : movement(movement)
{
}

const Movement* Movement_on_new_frame::get_original_movement() const
{
  return this->movement;
};

void Movement_on_new_frame::print(std::ostream& stream) const
{
  stream << "TODO !";
}

void Movement_on_new_frame::setSample(const MovementSample& samples)
{
  movement->setSample(samples);
}

const MovementSample& Movement_on_new_frame::getSample() const
{
  return movement->getSample();
}

rhoban_geometry::Point Movement_on_new_frame::linearPosition(double time) const
{
  return frame.toFrame(movement->linearPosition(time));
}

double Movement_on_new_frame::lastTime() const
{
  return movement->lastTime();
}

ContinuousAngle Movement_on_new_frame::angularPosition(double time) const
{
  return frame.toFrame(movement->angularPosition(time));
}

Vector2d Movement_on_new_frame::linearVelocity(double time) const
{
  return frame.toBasis(movement->linearVelocity(time));
}

ContinuousAngle Movement_on_new_frame::angularVelocity(double time) const
{
  return movement->angularVelocity(time);
}

Vector2d Movement_on_new_frame::linearAcceleration(double time) const
{
  return frame.toBasis(movement->linearAcceleration(time));
}

ContinuousAngle Movement_on_new_frame::angularAcceleration(double time) const
{
  return movement->angularAcceleration(time);
}

Movement* Movement_on_new_frame::clone() const
{
  Movement_on_new_frame* mov = new Movement_on_new_frame(movement->clone());
  mov->frame = this->frame;
  return mov;
}

Movement_on_new_frame::~Movement_on_new_frame()
{
  delete movement;
}

}  // namespace rhoban_ssl
