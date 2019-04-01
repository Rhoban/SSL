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

#pragma once

#include "robot_behavior.h"
#include "factory.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
class TestVelocityConsign : public RobotBehavior
{
private:
  ConsignFollower* follower_;
  rhoban_ssl::annotations::Annotations annotations_;

  double period_;
  double last_time_;
  int cpt_;

  Vector2d linear_velocity_;
  ContinuousAngle angular_velocity_;

public:
  TestVelocityConsign(ai::AiData& ai_data);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  static TestVelocityConsign* getWMovement(ai::AiData& ai_data, double velocity);
  static TestVelocityConsign* getEMovement(ai::AiData& ai_data, double velocity);
  static TestVelocityConsign* getNMovement(ai::AiData& ai_data, double velocity);
  static TestVelocityConsign* getSMovement(ai::AiData& ai_data, double velocity);
  static TestVelocityConsign* getNWMovement(ai::AiData& ai_data, double velocity);
  static TestVelocityConsign* getNEMovement(ai::AiData& ai_data, double velocity);
  static TestVelocityConsign* getSWMovement(ai::AiData& ai_data, double velocity);
  static TestVelocityConsign* getSEMovement(ai::AiData& ai_data, double velocity);

  void setLinearVelocity(const Vector2d& velocity);
  void setAngularVelocity(const ContinuousAngle& angle);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~TestVelocityConsign();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
