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

#include "pid.h"
#include <rhoban_utils/angle.h>
#include <math/matrix2d.h>

#define CALCULUS_ERROR 0.000
#define LIMITE 200.0
#define LIMITE_ROT 3.0

PidController::PidController() : PidController(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
{
}
PidController::PidController(double p_t, double i_t, double d_t, double p_o, double i_o, double d_o)
  : kp_t(p_t)
  , ki_t(i_t)
  , kd_t(d_t)
  , kp_o(p_o)
  , ki_o(i_o)
  , kd_o(d_o)
  , static_robot(true)
  , start_time(0.0)
  , time(0.0)
  , dt(0.0)
  , acc(0.0, 0.0)
  , acc_r(0.0)
  , ancient_pos(0.0, 0.0)
  , ancient_orientation(0.0)
{
}

void PidController::setStatic(bool value = true)
{
  static_robot = value;
}
bool PidController::isStatic() const
{
  return static_robot;
}

double PidController::getDt() const
{
  return dt;
}

void PidController::initTime(double start_time, double dt)
{
  // dt == 0 only at ai_data initialization.
  assert(dt >= 0.0);
  this->start_time = start_time;
  this->dt = dt;
  this->time = 0.0;
}

void PidController::setOrientationPid(double kp, double ki = 0.0, double kd = 0.0)
{
  this->kp_o = kp;
  this->ki_o = ki;
  this->kd_o = kd;
}

void PidController::setTranslationPid(double kp, double ki = 0.0, double kd = 0.0)
{
  this->kp_t = kp;
  this->ki_t = ki;
  this->kd_t = kd;
}

void PidController::update(double current_time, const Vector2d& robot_position,
                           const ContinuousAngle& robot_orientation)
{
  double dt = (current_time - start_time) - this->time;
  if (dt > 0.0)
  {
    this->dt = (current_time - start_time) - this->time;
    this->time = (current_time - start_time);
  }

  computeNoLimitedTranslationControl(robot_position);
  computeNoLimitedAngularControl(robot_orientation);
}

double PidController::getTime() const
{
  return this->time;
}

Vector2d PidController::noLimitedTranslationControl() const
{
  return no_limited_translation_control_value;
}

void PidController::computeNoLimitedTranslationControl(const Vector2d& robot_position)
{
  assert(dt > 0);
  if (isStatic())
  {
    no_limited_translation_control_value = Vector2d(0.0, 0.0);
    return;
  }
  Vector2d xt = goalPosition(time);
  // Vector2d xt_dt = goal_position(time + dt);
  Vector2d velocity = (robot_position - ancient_pos) / dt;

  Vector2d error = xt - robot_position;

  if (std::fabs(error[0]) < CALCULUS_ERROR and std::fabs(error[1]) < CALCULUS_ERROR)
  {
    error = Vector2d(0.0, 0.0);
  }
  // DEBUG("error: " << error );
  // std::cout<<time<<" "<<error[0]<<" "<<error[1]<<std::endl;

  acc_dt[0] = acc[0] - ki_t * error[0] * dt;
  if (acc_dt[0] > LIMITE)
  {
    acc[0] = LIMITE;
  }
  else if (acc_dt[0] < -LIMITE)
  {
    acc[0] = -LIMITE;
  }
  else
  {
    acc[0] = acc_dt[0];
  }

  acc_dt[1] = acc[1] - ki_t * error[1] * dt;
  if (acc_dt[1] > LIMITE)
  {
    acc[1] = LIMITE;
  }
  else if (acc_dt[1] < -LIMITE)
  {
    acc[1] = -LIMITE;
  }
  else
  {
    acc[1] = acc_dt[1];
  }

  no_limited_translation_control_value = (kp_t * error + acc + kd_t * velocity);

  // DEBUG( "PID" << time<<" "<<no_limited_translation_control_value[0]<<" "<<no_limited_translation_control_value[1]);

  ancient_pos = robot_position;
}

void PidController::computeNoLimitedAngularControl(const ContinuousAngle& robot_orientation)
{
  if (isStatic())
  {
    no_limited_angular_control_value = 0.0;
    return;
  };
  ContinuousAngle theta_t = goalOrientation(time);
  // ContinuousAngle theta_t_dt = goal_orientation(time + dt);
  // DEBUG( "theta_t : " << theta_t );
  // DEBUG( "theta_t_dt : " << theta_t_dt );
  ContinuousAngle velocity = (robot_orientation - ancient_orientation) / dt;
  ContinuousAngle error = theta_t - robot_orientation;
  // DEBUG("velocity : " << velocity );
  // DEBUG("theta_t: " << theta_t );
  // DEBUG("robot_orientation: " << robot_orientation );
  // DEBUG("error: " << error );

  if (std::fabs(error.value()) <= CALCULUS_ERROR)
  {
    // DEBUG("ERROR SET TO 0");
    error = 0.0;
  }
  acc_r_dt = acc_r - ki_o * error.value() * dt;
  if (acc_r_dt > LIMITE_ROT)
  {
    acc_r = LIMITE_ROT;
  }
  else if (acc_r_dt < -LIMITE_ROT)
  {
    acc_r = -LIMITE_ROT;
  }
  else
  {
    acc_r = acc_r_dt;
  }

  no_limited_angular_control_value = (kp_o * error.value() + acc_r + kd_o * velocity.value());
  ancient_orientation = robot_orientation;

  // DEBUG( "kpt : " << kp_o );
}

double PidController::noLimitedAngularControl() const
{
  return no_limited_angular_control_value;
}

Control PidController::noLimitedControl() const
{
  return Control(noLimitedTranslationControl(), noLimitedAngularControl());
}

PidController::~PidController()
{
}
