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

#ifndef __ROBOT_CONTROL_WITH_CURVE__H__
#define __ROBOT_CONTROL_WITH_CURVE__H__

#include "robot_control.h"

class RobotControlWithCurve : public RobotControlWithPid
{
public:
  CurveForRobot curve;

  RobotControlWithCurve();

  void set_movement(const std::function<Vector2d(double u)>& translation, double translation_velocity,
                    double translation_acceleration, const std::function<double(double u)>& rotation,
                    double angular_velocity, double angular_acceleration, double calculus_step, double current_time,
                    double current_dt);

  ContinuousAngle goal_orientation(double t) const;
  Vector2d goal_position(double t) const;
};

#endif
