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

#include "test_velocity_consign.h"
#include <math/vector2d.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
#define PERIOD 10.0

Test_velocity_consign::Test_velocity_consign(Ai::AiData& ai_data) : RobotBehavior(ai_data)
{
}

void Test_velocity_consign::set_angular_velocity(const ContinuousAngle& angular_velocity)
{
  this->angular_velocity = angular_velocity;
}

void Test_velocity_consign::set_linear_velocity(const Vector2d& linear_velocity)
{
  this->linear_velocity = linear_velocity;
}

void Test_velocity_consign::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
}

Control Test_velocity_consign::control() const
{
  Control ctrl;
  ctrl.linear_velocity = linear_velocity;
  ctrl.angular_velocity = angular_velocity;
  return ctrl;
}

Test_velocity_consign::~Test_velocity_consign()
{
}

RhobanSSLAnnotation::Annotations Test_velocity_consign::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addArrow(linear_position(), linear_position() + linear_velocity, "blue");
  annotations.addCircle(linear_position(), std::fabs(angular_velocity.value()), "blue");
  return annotations;
}

Test_velocity_consign* Test_velocity_consign::get_W_movement(Ai::AiData& ai_data, double velocity)
{
  Test_velocity_consign* res = new Test_velocity_consign(ai_data);
  res->set_linear_velocity(Vector2d(-1.0, 0.0) * velocity);
  return res;
}
Test_velocity_consign* Test_velocity_consign::get_E_movement(Ai::AiData& ai_data, double velocity)
{
  Test_velocity_consign* res = new Test_velocity_consign(ai_data);
  res->set_linear_velocity(Vector2d(1.0, 0.0) * velocity);
  return res;
}
Test_velocity_consign* Test_velocity_consign::get_N_movement(Ai::AiData& ai_data, double velocity)
{
  Test_velocity_consign* res = new Test_velocity_consign(ai_data);
  res->set_linear_velocity(Vector2d(0.0, 1.0) * velocity);
  return res;
}
Test_velocity_consign* Test_velocity_consign::get_S_movement(Ai::AiData& ai_data, double velocity)
{
  Test_velocity_consign* res = new Test_velocity_consign(ai_data);
  res->set_linear_velocity(Vector2d(0.0, -1.0) * velocity);
  return res;
}
Test_velocity_consign* Test_velocity_consign::get_NW_movement(Ai::AiData& ai_data, double velocity)
{
  Test_velocity_consign* res = new Test_velocity_consign(ai_data);
  res->set_linear_velocity(Vector2d(-1.0, 1.0) * velocity);
  return res;
}
Test_velocity_consign* Test_velocity_consign::get_NE_movement(Ai::AiData& ai_data, double velocity)
{
  Test_velocity_consign* res = new Test_velocity_consign(ai_data);
  res->set_linear_velocity(Vector2d(1.0, 1.0) * velocity);
  return res;
}
Test_velocity_consign* Test_velocity_consign::get_SW_movement(Ai::AiData& ai_data, double velocity)
{
  Test_velocity_consign* res = new Test_velocity_consign(ai_data);
  res->set_linear_velocity(Vector2d(-1.0, -1.0) * velocity);
  return res;
}
Test_velocity_consign* Test_velocity_consign::get_SE_movement(Ai::AiData& ai_data, double velocity)
{
  Test_velocity_consign* res = new Test_velocity_consign(ai_data);
  res->set_linear_velocity(Vector2d(1.0, -1.0) * velocity);
  return res;
}

}  // namespace Robot_behavior
}  // namespace RhobanSSL
