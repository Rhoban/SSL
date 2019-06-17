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
#include "consign_follower.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
class PositionFollower : public ConsignFollower
{
private:
  rhoban_geometry::Point position_;
  ContinuousAngle angle_;

  RobotControlWithPositionFollowing robot_control_;

public:
  PositionFollower(double time, double dt);

  void setTranslationPid(double kp, double ki, double kd);
  void setOrientationPid(double kp, double ki, double kd);

  void setLimits(double translation_velocity_limit, double rotation_velocity_limit,
                 double translation_acceleration_limit, double rotation_acceleration_limit);

  virtual void setFollowingPosition(const rhoban_geometry::Point& position_to_follow, const ContinuousAngle& angle);

protected:
  void updateControl(double time);

public:
  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  virtual Control control() const;
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
