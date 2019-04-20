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
void MovementOnNewFrame::setFrame(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2)
{
  frame_.setFrame(origin, v1, v2);
}

MovementOnNewFrame::MovementOnNewFrame(Movement* movement) : movement_(movement)
{
}

const Movement* MovementOnNewFrame::getOriginalMovement() const
{
  return movement_;
};

void MovementOnNewFrame::print(std::ostream& stream) const
{
  stream << "TODO !";
}

void MovementOnNewFrame::setSample(const MovementSample& samples)
{
  movement_->setSample(samples);
}

const MovementSample& MovementOnNewFrame::getSample() const
{
  return movement_->getSample();
}

rhoban_geometry::Point MovementOnNewFrame::linearPosition(double time) const
{
  return frame_.toFrame(movement_->linearPosition(time));
}

double MovementOnNewFrame::lastTime() const
{
  return movement_->lastTime();
}

ContinuousAngle MovementOnNewFrame::angularPosition(double time) const
{
  return frame_.toFrame(movement_->angularPosition(time));
}

Vector2d MovementOnNewFrame::linearVelocity(double time) const
{
  return frame_.toBasis(movement_->linearVelocity(time));
}

ContinuousAngle MovementOnNewFrame::angularVelocity(double time) const
{
  return movement_->angularVelocity(time);
}

Vector2d MovementOnNewFrame::linearAcceleration(double time) const
{
  return frame_.toBasis(movement_->linearAcceleration(time));
}

ContinuousAngle MovementOnNewFrame::angularAcceleration(double time) const
{
  return movement_->angularAcceleration(time);
}

Movement* MovementOnNewFrame::clone() const
{
  MovementOnNewFrame* mov = new MovementOnNewFrame(movement_->clone());
  mov->frame_ = frame_;
  return mov;
}

MovementOnNewFrame::~MovementOnNewFrame()
{
  delete movement_;
}

}  // namespace rhoban_ssl
