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
PositionFollowerWithTargetTracking::PositionFollowerWithTargetTracking(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
{
  initTime(ai_data.time, ai_data.dt);
}


void PositionFollowerWithTargetTracking::update(double time, const ai::Robot& robot,
                                                const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  RobotControlWithTargetTrackingAndPositionFollowing::update(time,
                                                             robot.getMovement().linearPosition(time),
                                                             robot.getMovement().linearVelocity(time),
                                                             robot.getMovement().angularPosition(time));
}

Control PositionFollowerWithTargetTracking::control() const
{
  return getLimitedControl();
}

annotations::Annotations PositionFollowerWithTargetTracking::getAnnotations() const
{
  annotations::Annotations annotations;
  bool dashed = true;

  annotations.addArrow(RobotBehavior::linearPosition(), linear_position_of_the_target_, "red", dashed);

  annotations.addArrow(RobotBehavior::linearPosition(),
                       RobotBehavior::linearPosition() +
                           Vector2d(std::cos(RobotBehavior::angularPosition().value()), std::sin(RobotBehavior::angularPosition().value())),
                       "red", not dashed);

  //    annotations.addArrow(
  //        linear_position(), m_positionToFollow, "magenta", dashed
  //    );

  Control ctrl = control();
  annotations.addArrow(RobotBehavior::linearPosition(), RobotBehavior::linearPosition() + ctrl.linear_velocity, "orange", not dashed);
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
