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
namespace Robot_behavior
{
TestInfra::TestInfra(ai::AiData& ai_data) : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data))
{
}

void TestInfra::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  rhoban_geometry::Point target_position = robot.getMovement().linearPosition(time);

  bool value = GameInformations::infraRed(robot.id(), vision::Team::Ally);
  std::cout << "Value infra red : " << value << '\n';

  // follower->avoid_the_ball(true);
  double target_rotation = detail::vec2angle(ballPosition() - target_position);

  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control TestInfra::control() const
{
  Control ctrl = follower->control();
  return ctrl;
}

TestInfra::~TestInfra()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations TestInfra::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
