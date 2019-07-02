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

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
#define PERIOD 10.0

TestVelocityConsign::TestVelocityConsign() : RobotBehavior()
{
}

void TestVelocityConsign::setAngularVelocity(const ContinuousAngle& angular_velocity)
{
  this->angular_velocity_ = angular_velocity;
}

void TestVelocityConsign::setLinearVelocity(const Vector2d& linear_velocity)
{
  this->linear_velocity_ = linear_velocity;
}

void TestVelocityConsign::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
}

Control TestVelocityConsign::control() const
{
  Control ctrl;
  ctrl.linear_velocity = linear_velocity_;
  ctrl.angular_velocity = angular_velocity_;
  return ctrl;
}

TestVelocityConsign::~TestVelocityConsign()
{
}

rhoban_ssl::annotations::Annotations TestVelocityConsign::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addArrow(linearPosition(), linearPosition() + linear_velocity_, "blue");
  annotations.addCircle(linearPosition(), std::fabs(angular_velocity_.value()), "blue");
  return annotations;
}

TestVelocityConsign* TestVelocityConsign::getWMovement(double velocity)
{
  TestVelocityConsign* res = new TestVelocityConsign();
  res->setLinearVelocity(Vector2d(-1.0, 0.0) * velocity);
  return res;
}
TestVelocityConsign* TestVelocityConsign::getEMovement(double velocity)
{
  TestVelocityConsign* res = new TestVelocityConsign();
  res->setLinearVelocity(Vector2d(1.0, 0.0) * velocity);
  return res;
}
TestVelocityConsign* TestVelocityConsign::getNMovement(double velocity)
{
  TestVelocityConsign* res = new TestVelocityConsign();
  res->setLinearVelocity(Vector2d(0.0, 1.0) * velocity);
  return res;
}
TestVelocityConsign* TestVelocityConsign::getSMovement(double velocity)
{
  TestVelocityConsign* res = new TestVelocityConsign();
  res->setLinearVelocity(Vector2d(0.0, -1.0) * velocity);
  return res;
}
TestVelocityConsign* TestVelocityConsign::getNWMovement(double velocity)
{
  TestVelocityConsign* res = new TestVelocityConsign();
  res->setLinearVelocity(Vector2d(-1.0, 1.0) * velocity);
  return res;
}
TestVelocityConsign* TestVelocityConsign::getNEMovement(double velocity)
{
  TestVelocityConsign* res = new TestVelocityConsign();
  res->setLinearVelocity(Vector2d(1.0, 1.0) * velocity);
  return res;
}
TestVelocityConsign* TestVelocityConsign::getSWMovement(double velocity)
{
  TestVelocityConsign* res = new TestVelocityConsign();
  res->setLinearVelocity(Vector2d(-1.0, -1.0) * velocity);
  return res;
}
TestVelocityConsign* TestVelocityConsign::getSEMovement(double velocity)
{
  TestVelocityConsign* res = new TestVelocityConsign();
  res->setLinearVelocity(Vector2d(1.0, -1.0) * velocity);
  return res;
}

}  // namespace tests
}  // namespace robot_behavior
}  // namespace rhoban_ssl
