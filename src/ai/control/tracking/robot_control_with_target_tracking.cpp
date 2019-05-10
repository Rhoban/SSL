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
#include "robot_control_with_target_tracking.h"

#include <math/matrix2d.h>
#include <debug.h>

namespace rhoban_ssl
{
namespace robot_control
{
namespace tracking
{
Vector2d RobotControlWithTargetTracking::setLinearVelocityOfTheTarget(const Vector2d& linear_velocity_of_the_target)
{
  linear_velocity_of_the_target_ = linear_velocity_of_the_target;
}

Vector2d RobotControlWithTargetTracking::setLinearPositionOfTheTarget(
    const rhoban_geometry::Point& linear_position_of_the_target)
{
  linear_position_of_the_target_ = linear_position_of_the_target;
}

ContinuousAngle RobotControlWithTargetTracking::angularVelocity(double time) const
{
  Vector2d robot_target_absolute = linear_position_of_the_target_ - current_linear_position_;
  Vector2d robot_target_relative = /* rotation_matrix * */ robot_target_absolute;
  Vector2d normalized_robot_target_relative = robot_target_relative / robot_target_relative.norm();

  return vectorialProduct(normalized_robot_target_relative,
                           (linear_velocity_of_the_target_ - current_linear_velocity_) / robot_target_relative.norm());
}

ContinuousAngle RobotControlWithTargetTracking::angularPosition(double time) const
{
  Vector2d robot_target = linear_position_of_the_target_ - linearPosition(time);
  return vector2angle(robot_target);
}

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl
