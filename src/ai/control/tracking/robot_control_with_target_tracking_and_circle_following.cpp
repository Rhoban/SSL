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
#include "robot_control_with_target_tracking_and_circle_following.h"

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
void RobotControlWithTargetTrackingAndCircleFollowing::setGoal(
    const ContinuousAngle& robot_destination, const rhoban_geometry::Point& linear_position_of_the_target,
    const Vector2d& linear_velocity_of_the_target)
{
  robot_destination_ = robot_destination;
  RobotControlWithTargetTracking::setLinearPositionOfTheTarget(linear_position_of_the_target);
  RobotControlWithTargetTracking::setLinearVelocityOfTheTarget(linear_velocity_of_the_target);
  setStatic(false);
}

Vector2d RobotControlWithTargetTrackingAndCircleFollowing::linearVelocity(double) const
{
//  Vector2d robot_target = linear_position_of_the_target_ - linearPosition(time_);
//  Vector2d robot_target_orth(robot_target.getY(), -1*robot_target.getX());

//  //TODO change sign according to the distance from targeted angle

//  robot_target_orth/robot_target_orth.norm()*NORM_OF_THE_VELOCITY;

  return {0,0};
}

Vector2d RobotControlWithTargetTrackingAndCircleFollowing::linearPosition(double time) const
{
  //return linear_position_at_start_ + linearVelocity(0) * (time_);
//  double dx = current_linear_position_.getX() - linear_position_of_the_target_.getX();
//  double dy = current_linear_position_.getY() - linear_position_of_the_target_.getY();
//  double radius_of_trajectory =  sqrt(dx * dx + dy * dy);

//  ContinuousAngle angular_position_in_trajectory = linearVelocity(time_).norm() / radius_of_trajectory;

//  return Vector2d(std::cos(angular_position_in_trajectory.value()), std::sin(angular_position_in_trajectory.value()))  ;
  return {0,0};
}

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl
