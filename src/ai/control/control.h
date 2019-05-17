/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)
    Copyright 2019 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)


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

#include <math/continuous_angle.h>
#include <math/vector2d.h>
#include <execution_manager.h>
#include <com/ai_commander.h>

class Control
{
private:
  bool is_absolute_;

public:
  // TODO : REFACTOR THIS PART ?
  // fix_translation and fix_rotation is used to set odometry: to give the absolute position to the robot.
  Vector2d fix_translation = Vector2d(0, 0);  // References for Odometry
  ContinuousAngle fix_rotation = ContinuousAngle(0);

  Vector2d linear_velocity = Vector2d(0, 0);
  ContinuousAngle angular_velocity = ContinuousAngle(0);

  bool charge = false;
  bool kick = false;
  bool chipKick = false;
  float kickPower = 1.0;
  bool spin = false;

  bool active = true;
  bool ignore = false;
  bool tareOdom = false;  // Reset references for Odometry robot

  Control(bool isAbsolute = true);

  Control(const Vector2d& linear_velocity, const ContinuousAngle& angular_velocity, bool isAbsolute = true);

  Control(bool kick, bool active, bool ignore);

  void changeToRelativeControl(const ContinuousAngle& robot_orientation, double dt);

  void changeToAbsoluteControl(const ContinuousAngle& robot_orientation, double dt);

  bool isAbsolute();

  bool isRelative();

  static Control makeDesactivated();
  static Control makeIgnored();
  static Control makeNull();
};

std::ostream& operator<<(std::ostream& out, const Control& control);

namespace rhoban_ssl
{
class ControlSender : public Task
{
private:
  AICommander* commander_;
public:
  ControlSender(AICommander* commander);
  // Task interface
public:
  bool runTask();
};

}  // namespace rhoban_ssl
