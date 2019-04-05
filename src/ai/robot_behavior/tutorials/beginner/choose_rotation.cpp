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

#include "choose_rotation.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace beginner
{
ChooseRotation::ChooseRotation(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data))
{
}

void ChooseRotation::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  // We declare and set the rotation goal at PI/2 radian
  ContinuousAngle rotation_goal(M_PI / 2);

  // We get the current position of the robot
  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);

  // We send the position and the rotation goal to
  // the ConsignFollower
  follower_->setFollowingPosition(robot_position, rotation_goal);

  // We apply the new instruction with the call of the
  // update method of the ConsignFollower
  follower_->update(time, robot, ball);
}

Control ChooseRotation::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

ChooseRotation::~ChooseRotation()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations ChooseRotation::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

};  // namespace beginner
};  // namespace robot_behavior
};  // namespace rhoban_ssl
