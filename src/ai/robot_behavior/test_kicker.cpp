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
namespace Robot_behavior
{
Test_kicker::Test_kicker(Ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data))
{
}

void Test_kicker::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);

  annotations.clear();

  const rhoban_geometry::Point& target_position = centerAllyField();
  ContinuousAngle target_rotation(M_PI);

  follower->avoid_the_ball(false);
  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Test_kicker::control() const
{
  Control ctrl = follower->control();
  ctrl.kick = true;
  ctrl.charge = true;
  return ctrl;
}

Test_kicker::~Test_kicker()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Test_kicker::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  annotations.addAnnotations(follower->get_annotations());
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
