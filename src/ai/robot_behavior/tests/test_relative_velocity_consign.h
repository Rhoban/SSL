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

#pragma once

#include "../robot_behavior.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
class TestRelativeVelocityConsign : public RobotBehavior
{
private:
  Control relative_control_;

public:
  TestRelativeVelocityConsign();
  virtual ~TestRelativeVelocityConsign();

  void setLinearVelocity(const Vector2d& linear_velocity);
  void setAngularVelocity(const ContinuousAngle& angular_velocity);

  // RobotBehavior interface
public:
  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);
  virtual Control control() const;
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  // Tests
public:
  static TestRelativeVelocityConsign* getMovementAngularVelocityOnly(double angular_velocity);
  static TestRelativeVelocityConsign* getMovementLinearVelocityOnly(Vector2d linear_velocity);
  static TestRelativeVelocityConsign* getMovementAngularAndLinearVelocity(Vector2d linear_velocity,
                                                                          double angular_velocity);
};

};  // namespace tests
};  // namespace robot_behavior
};  // namespace rhoban_ssl