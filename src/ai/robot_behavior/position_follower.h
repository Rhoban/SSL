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

namespace RhobanSSL
{
namespace Robot_behavior
{
class PositionFollower : public ConsignFollower
{
private:
  rhoban_geometry::Point position;
  ContinuousAngle angle;

  RobotControlWithPositionFollowing robot_control;

public:
  PositionFollower(Ai::AiData& ai_data, double time, double dt);

  void set_translation_pid(double kp, double ki, double kd);
  void set_orientation_pid(double kp, double ki, double kd);

  void set_limits(double translation_velocity_limit, double rotation_velocity_limit,
                  double translation_acceleration_limit, double rotation_acceleration_limit);

  virtual void set_following_position(const rhoban_geometry::Point& position_to_follow, const ContinuousAngle& angle);

protected:
  void update_control(double time);

public:
  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  virtual Control control() const;
  virtual RhobanSSLAnnotation::Annotations get_annotations() const;
};

};  // namespace Robot_behavior
};  // namespace RhobanSSL
