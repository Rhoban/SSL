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
  frame.set_frame(origin, v1, v2);
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

void Movement_on_new_frame::set_sample(const MovementSample& samples)
{
  movement->set_sample(samples);
}

const MovementSample& Movement_on_new_frame::get_sample() const
{
  return movement->get_sample();
}

rhoban_geometry::Point Movement_on_new_frame::linear_position(double time) const
{
  return frame.to_frame(movement->linear_position(time));
}

double Movement_on_new_frame::last_time() const
{
  return movement->last_time();
}

ContinuousAngle Movement_on_new_frame::angular_position(double time) const
{
  return frame.to_frame(movement->angular_position(time));
}

Vector2d Movement_on_new_frame::linear_velocity(double time) const
{
  return frame.to_basis(movement->linear_velocity(time));
}

ContinuousAngle Movement_on_new_frame::angular_velocity(double time) const
{
  return movement->angular_velocity(time);
}

Vector2d Movement_on_new_frame::linear_acceleration(double time) const
{
  return frame.to_basis(movement->linear_acceleration(time));
}

ContinuousAngle Movement_on_new_frame::angular_acceleration(double time) const
{
  return movement->angular_acceleration(time);
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
