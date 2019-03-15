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

#ifndef __pid__H__
#define __pid__H__

#include "control.h"

#include <debug.h>
#include <math/ContinuousAngle.h>
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

  void init_time(double start_time, double dt);

  void update(double current_time, const Vector2d& robot_position, const ContinuousAngle& robot_orientation);

  PidController();
  PidController(double p_t, double i_t, double d_t, double p_o, double i_o, double d_o);

  void set_orientation_pid(double kp, double ki, double kd);
  void set_translation_pid(double kp, double ki, double kd);

  void set_static(bool value);
  bool is_static() const;

  double get_dt() const;
  double get_time() const;

  virtual ContinuousAngle goal_orientation(double t) const = 0;
  virtual Vector2d goal_position(double t) const = 0;

  void compute_no_limited_translation_control(const Vector2d& robot_position);
  Vector2d no_limited_translation_control() const;

  void compute_no_limited_angular_control(const ContinuousAngle& robot_orientation);
  double no_limited_angular_control() const;

  virtual Control no_limited_control() const;
};

#endif
