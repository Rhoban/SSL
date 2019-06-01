/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

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
#include "robot_control.h"
#include <debug.h>

namespace rhoban_ssl
{
namespace robot_control
{
namespace tracking
{
void RobotControl::update(double current_time, const Vector2d& robot_linear_position,
                          const Vector2d& robot_linear_velocity, const ContinuousAngle& robot_angular_position)
{
  if (!init && current_linear_position_ != robot_linear_position)
  {
    linear_position_at_start_ = robot_linear_position;
    angular_position_at_start_ = robot_angular_position;

    const double DELAY_ROBOT_AI = 0.01;
    initTime(current_time + DELAY_ROBOT_AI, dt_);
    init = true;
  }

  double dt = (current_time - start_time_) - time_;
  if (dt > 0.0)
  {
    dt_ = (current_time - start_time_) - time_;
    time_ = (current_time - start_time_);
  }

  // DEBUG(time_);

  current_angular_position_ = robot_angular_position;
  current_linear_velocity_ = robot_linear_velocity;
  current_linear_position_ = robot_linear_position;

  // DEBUG("reel " << current_linear_velocity_.norm());

  computeLimitedControl();
}

void RobotControl::setStatic(bool value)
{
  static_robot_ = value;
}

bool RobotControl::isStatic() const
{
  return static_robot_;
}

void RobotControl::initTime(double start_time, double dt)
{
  DEBUG("coucou");
  assert(dt > 0.0);
  start_time_ = start_time;
  dt_ = dt;
  time_ = 0.0;
}

double RobotControl::getDt() const
{
  return dt_;
}

double RobotControl::getTime() const
{
  return time_;
}

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl
