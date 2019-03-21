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
#include "position_follower_with_target_tracking.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
PositionFollowerWithTargetTracking::PositionFollowerWithTargetTracking(RhobanSSL::Ai::AiData& ai_data)
  : RhobanSSL::Robot_behavior::RobotBehavior(ai_data)
{
}

void PositionFollowerWithTargetTracking::update(double time, const RhobanSSL::Ai::Robot& robot,
                                                const RhobanSSL::Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);

  //TODO MOVE TO RobotControlWithTargetTrackingAndPositionFollowing
  if (!isStatic() && !is_time_init_)
  {
    this->initTime(time, ai_data.dt);
    is_time_init_ = true;
  }

  RobotControlWithTargetTrackingAndPositionFollowing::update(time,
                                                             robot.get_movement().linear_position(time),
                                                             robot.get_movement().linear_velocity(time),
                                                             robot.get_movement().angular_position(time));
}

Control PositionFollowerWithTargetTracking::control() const
{
  return getLimitedControl();
}

RhobanSSLAnnotation::Annotations PositionFollowerWithTargetTracking::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  bool dashed = true;

  annotations.addArrow(linear_position(), linear_position_of_the_target_, "red", dashed);

  annotations.addArrow(linear_position(),
                       linear_position() +
                           Vector2d(std::cos(angular_position().value()), std::sin(angular_position().value())),
                       "red", not dashed);

  //    annotations.addArrow(
  //        linear_position(), m_positionToFollow, "magenta", dashed
  //    );

  Control ctrl = control();
  annotations.addArrow(linear_position(), linear_position() + ctrl.linear_velocity, "orange", not dashed);
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
