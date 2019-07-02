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

#include "../robot_behavior.h"
#include "../factory.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
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
  TestVelocityConsign();

  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  static TestVelocityConsign* getWMovement(double velocity);
  static TestVelocityConsign* getEMovement(double velocity);
  static TestVelocityConsign* getNMovement(double velocity);
  static TestVelocityConsign* getSMovement(double velocity);
  static TestVelocityConsign* getNWMovement(double velocity);
  static TestVelocityConsign* getNEMovement(double velocity);
  static TestVelocityConsign* getSWMovement(double velocity);
  static TestVelocityConsign* getSEMovement(double velocity);

  void setLinearVelocity(const Vector2d& velocity);
  void setAngularVelocity(const ContinuousAngle& angle);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~TestVelocityConsign();
};

};  // namespace tests
};  // namespace robot_behavior
};  // namespace rhoban_ssl
