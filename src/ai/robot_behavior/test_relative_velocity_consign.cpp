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
namespace robot_behavior
{
TestRelativeVelocityConsign::TestRelativeVelocityConsign(rhoban_ssl::ai::AiData& ai_data)
  : RobotBehavior(ai_data), relative_control_(false)
{
}

TestRelativeVelocityConsign::~TestRelativeVelocityConsign()
{
}

void TestRelativeVelocityConsign::setLinearVelocity(const Vector2d& linear_velocity)
{
  relative_control_.linear_velocity = linear_velocity;
}

void TestRelativeVelocityConsign::setAngularVelocity(const ContinuousAngle& angular_velocity)
{
  relative_control_.angular_velocity = angular_velocity;
}

void TestRelativeVelocityConsign::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
}

Control TestRelativeVelocityConsign::control() const
{
  return relative_control_;
}

rhoban_ssl::annotations::Annotations TestRelativeVelocityConsign::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addArrow(linearPosition(), linearPosition() + relative_control_.linear_velocity, "blue");
  return annotations;
}

TestRelativeVelocityConsign*
TestRelativeVelocityConsign::getMovementAngularVelocityOnly(ai::AiData& ai_data, double angular_velocity)
{
  TestRelativeVelocityConsign* res = new TestRelativeVelocityConsign(ai_data);
  res->setAngularVelocity(angular_velocity);
  return res;
}

TestRelativeVelocityConsign*
TestRelativeVelocityConsign::getMovementLinearVelocityOnly(ai::AiData& ai_data, Vector2d linear_velocity)
{
  TestRelativeVelocityConsign* res = new TestRelativeVelocityConsign(ai_data);
  res->setLinearVelocity(linear_velocity);
  return res;
}

TestRelativeVelocityConsign* TestRelativeVelocityConsign::getMovementAngularAndLinearVelocity(
    ai::AiData& ai_data, Vector2d linear_velocity, double angular_velocity)
{
  TestRelativeVelocityConsign* res = new TestRelativeVelocityConsign(ai_data);
  res->setLinearVelocity(linear_velocity);
  res->setAngularVelocity(angular_velocity);
  return res;
}

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
