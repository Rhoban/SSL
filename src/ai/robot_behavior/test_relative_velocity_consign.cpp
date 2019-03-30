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
#include "test_relative_velocity_consign.h"

namespace rhoban_ssl
{
namespace Robot_behavior
{
Test_relative_velocity_consign::Test_relative_velocity_consign(rhoban_ssl::Ai::AiData& ai_data)
  : RobotBehavior(ai_data), relative_control(false)
{
}

Test_relative_velocity_consign::~Test_relative_velocity_consign()
{
}

void Test_relative_velocity_consign::set_linear_velocity(const Vector2d& linear_velocity)
{
  relative_control.linear_velocity = linear_velocity;
}

void Test_relative_velocity_consign::set_angular_velocity(const ContinuousAngle& angular_velocity)
{
  relative_control.angular_velocity = angular_velocity;
}

void Test_relative_velocity_consign::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
}

Control Test_relative_velocity_consign::control() const
{
  return relative_control;
}

RhobanSSLAnnotation::Annotations Test_relative_velocity_consign::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addArrow(linear_position(), linear_position() + relative_control.linear_velocity, "blue");
  return annotations;
}

Test_relative_velocity_consign*
Test_relative_velocity_consign::get_movement_angular_velocity_only(Ai::AiData& ai_data, double angular_velocity)
{
  Test_relative_velocity_consign* res = new Test_relative_velocity_consign(ai_data);
  res->set_angular_velocity(angular_velocity);
  return res;
}

Test_relative_velocity_consign*
Test_relative_velocity_consign::get_movement_linear_velocity_only(Ai::AiData& ai_data, Vector2d linear_velocity)
{
  Test_relative_velocity_consign* res = new Test_relative_velocity_consign(ai_data);
  res->set_linear_velocity(linear_velocity);
  return res;
}

Test_relative_velocity_consign* Test_relative_velocity_consign::get_movement_angular_and_linear_velocity(
    Ai::AiData& ai_data, Vector2d linear_velocity, double angular_velocity)
{
  Test_relative_velocity_consign* res = new Test_relative_velocity_consign(ai_data);
  res->set_linear_velocity(linear_velocity);
  res->set_angular_velocity(angular_velocity);
  return res;
}

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
