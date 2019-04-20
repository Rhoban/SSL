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

#include "control.h"

#include <debug.h>
#include <math/continuous_angle.h>
#include <math/vector2d.h>
#include <utility>

struct PidController
{
  double kp_t, ki_t, kd_t;
  double kp_o, ki_o, kd_o;

  bool static_robot;

  double start_time;
  double time;
  double dt;

  Vector2d acc;
  Vector2d acc_dt;
  double acc_r;
  double acc_r_dt;

  Vector2d ancient_pos;
  ContinuousAngle ancient_orientation;

  double no_limited_angular_control_value;
  Vector2d no_limited_translation_control_value;

  void initTime(double start_time, double dt);

  void update(double current_time, const Vector2d& robot_position, const ContinuousAngle& robot_orientation);

  PidController();
  PidController(double p_t, double i_t, double d_t, double p_o, double i_o, double d_o);

  void setOrientationPid(double kp, double ki, double kd);
  void setTranslationPid(double kp, double ki, double kd);

  void setStatic(bool value);
  bool isStatic() const;

  double getDt() const;
  double getTime() const;

  virtual ContinuousAngle goalOrientation(double t) const = 0;
  virtual Vector2d goalPosition(double t) const = 0;

  void computeNoLimitedTranslationControl(const Vector2d& robot_position);
  Vector2d noLimitedTranslationControl() const;

  void computeNoLimitedAngularControl(const ContinuousAngle& robot_orientation);
  double noLimitedAngularControl() const;

  virtual Control noLimitedControl() const;
  virtual ~PidController();
};
