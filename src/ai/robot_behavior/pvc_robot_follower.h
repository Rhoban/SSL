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
class RobotFollower : public RobotBehavior
{
private:
  int robot_to_follow_id_;
  Team robot_to_follow_team_;

  Vector2d translation_;
  Team team_;

  ConsignFollower* follower_;

public:
  RobotFollower();

  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  void declareRobotToFollow(int robot_id, const Vector2d& translation, Team team = Ally);

  virtual Control control() const;

  virtual ~RobotFollower();
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
