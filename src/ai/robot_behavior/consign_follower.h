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

namespace rhoban_ssl
{
namespace robot_behavior
{
class ConsignFollower : public RobotBehavior
{
public:
  ConsignFollower();

  virtual void setFollowingPosition(const rhoban_geometry::Point& position_to_follow, const ContinuousAngle& angle) = 0;

  virtual void avoidTheBall(bool value);
  virtual void avoidAlly(bool value);
  virtual void avoidOpponent(bool value);
  virtual void avoidRobot(int id, bool value);

  virtual void setRadiusAvoidanceForTheBall(double radius);

  virtual ~ConsignFollower();
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
