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
#include "robot_control_with_target_tracking_and_position_following.h"

#include "debug.h"
/// TODO Replace after test with a value in config file
/// or
/// add an equation that take the distance between the robot's position and
/// the robot's destination and returns the velocity
#define NORM_OF_THE_VELOCITY 0.05

namespace rhoban_ssl
{
namespace robot_control
{
namespace tracking
{
void RobotControlWithTargetTrackingAndPositionFollowing::setGoal(
    const Vector2d& robot_destination, const rhoban_geometry::Point& linear_position_of_the_target,
    const Vector2d& linear_velocity_of_the_target)
{
  robot_destination_ = robot_destination;
  RobotControlWithTargetTracking::setLinearPositionOfTheTarget(linear_position_of_the_target);
  RobotControlWithTargetTracking::setLinearVelocityOfTheTarget(linear_velocity_of_the_target);
  setStatic(false);
}

Vector2d RobotControlWithTargetTrackingAndPositionFollowing::linearVelocity(double) const
{
  Vector2d linear_velocity = robot_destination_ - linear_position_at_start_;
  linear_velocity /= linear_velocity.norm();
  linear_velocity *= NORM_OF_THE_VELOCITY;


  return /*(current_linear_velocity_.norm() < 0.0000000001) ? */linear_velocity /*: current_linear_velocity_*/;
}

Vector2d RobotControlWithTargetTrackingAndPositionFollowing::linearPosition(double time) const
{
  return linear_position_at_start_ + linearVelocity(0) * (time_);
}

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl
