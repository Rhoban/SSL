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
#include "pid.h"

namespace rhoban_ssl
{
namespace robot_control
{
Pid::Pid(double p, double ki, double kd) : kp_(p), ki_(ki), kd_(kd)
{
}

Pid::Pid(double integration_limit, double p, double i, double d)
  : integration_limit_(integration_limit), kp_(p), ki_(i), kd_(d)
{
}

void Pid::setIntegrationLimit(double integration_limit)
{
  integration_limit_ = integration_limit;
}

void Pid::setPid(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void Pid::resetIntegration()
{
  sum_errors_ = 0.0;
}

void Pid::reset()
{
  last_error_ = 0.0;
  sum_errors_ = 0.0;
}

template <>
double Pid::compute<double>(const double& dt, const double& error)
{
  sum_errors_ += error * dt;

  double integral = ki_ * sum_errors_;
  if (integral > integration_limit_)
  {
    integral = integration_limit_;
  }
  else if (integral < integration_limit_)
  {
    integral = -integration_limit_;
  }

  double correction = kp_ * error + integral + kd_ * (error - last_error_) / dt;

  last_error_ = error;

  return correction;
}

template <>
Vector2d Pid::compute<Vector2d>(const double& dt, const Vector2d& error)
{
  double x = compute(dt, error[0]);
  double y = compute(dt, error[1]);
  return Vector2d(x, y);
}

template <>
ContinuousAngle Pid::compute<ContinuousAngle>(const double& dt, const ContinuousAngle& error)
{
  double angle = compute(dt, error.value());
  return ContinuousAngle(angle);
}

}  // namespace robot_control
}  // namespace rhoban_ssl
