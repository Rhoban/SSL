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

#include "test_infra.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <math/continuous_angle.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
TestInfra::TestInfra(ai::AiData& ai_data) : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data))
{
}

void TestInfra::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  rhoban_geometry::Point target_position = robot.getMovement().linearPosition(time);

  bool value = GameInformations::infraRed(robot.id(), vision::Team::Ally);
  std::cout << "Value infra red : " << value << '\n';

  // follower->avoid_the_ball(true);
  double target_rotation = detail::vec2angle(ballPosition() - target_position);

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control TestInfra::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

TestInfra::~TestInfra()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations TestInfra::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
