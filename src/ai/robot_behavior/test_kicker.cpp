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

#include "test_kicker.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
TestKicker::TestKicker(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data))
{
}

void TestKicker::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  const rhoban_geometry::Point& target_position = centerAllyField();
  ContinuousAngle target_rotation(M_PI);

  follower_->avoidTheBall(false);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control TestKicker::control() const
{
  Control ctrl = follower_->control();
  ctrl.kick = true;
  ctrl.charge = true;
  return ctrl;
}

TestKicker::~TestKicker()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations TestKicker::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
