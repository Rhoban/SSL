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

#include "robot_control.h"

#include <assert.h>
#include <math/curve.h>
#include "pid.h"
#include <math/matrix2d.h>
#include <algorithm>

#define CALCULUS_ERROR 0.00000001

CurveForRobot::CurveForRobot(const std::function<Vector2d(double u)>& translation, double translation_velocity,
                             double translation_acceleration, const std::function<double(double u)>& rotation,
                             double angular_velocity, double angular_acceleration, double calculus_step)
  : rotation_fct(rotation)
  , translation_curve(translation, calculus_step)
  , angular_curve(rotation_fct, calculus_step)
  , tranlsation_consign(translation_curve.size(), translation_velocity, translation_acceleration)
  , angular_consign(angular_curve.size(), angular_velocity, angular_acceleration)
  , calculus_error_for_translation(std::min(0.0001,  // 0.1 mm (in meter)
                                            calculus_step / 100))
  , calculus_error_for_rotation(std::min(0.0001,  // approx. 1/100 degree (in radian)
                                         calculus_step / 100))
  ,
  // TODO Calculet un time_step plus précisement
  translation_movment(translation_curve, tranlsation_consign, calculus_step, calculus_error_for_translation)
  , rotation_movment(angular_curve, angular_consign, calculus_step, calculus_error_for_rotation){};

Vector2d CurveForRobot::translation(double t) const
{
  return translation_movment(t);
}
double CurveForRobot::rotation(double t) const
{
  return rotation_movment(t)[0];
}

void CurveForRobot::printTranslationMovment(double dt) const
{
  std::cout << "translation movment : " << std::endl;
  std::cout << "--------------------- " << std::endl;
  double max_time = translation_movment.maxTime();
  std::cout << "   max time : " << max_time;
  for (double t = 0; t < max_time; t += dt)
  {
    Vector2d v = translation_movment(t);
    std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl;
  }
}

void CurveForRobot::printTranslationCurve(double dt) const
{
  std::cout << "translation curve : " << std::endl;
  std::cout << "------------------- " << std::endl;
  for (double t = 0; t < 1.0; t += dt)
  {
    Vector2d v = translation_curve(t);
    std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl;
  }
}

void CurveForRobot::printRotationMovment(double dt) const
{
  std::cout << "rotation movment : " << std::endl;
  std::cout << "--------------------- " << std::endl;
  double max_time = rotation_movment.maxTime();
  std::cout << "   max time : " << max_time;
  for (double t = 0; t < max_time; t += dt)
  {
    Vector2d v = rotation_movment(t);
    std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl;
  }
}

void CurveForRobot::printRotationCurve(double dt) const
{
  std::cout << "rotation curve : " << std::endl;
  std::cout << "------------------- " << std::endl;
  for (double t = 0; t < 1.0; t += dt)
  {
    Vector2d v = angular_curve(t);
    std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl;
  }
}

void RobotControl::setLimits(double translation_velocity_limit, double rotation_velocity_limit,
                              double translation_acceleration_limit, double rotation_acceleration_limit)
{
  this->translation_velocity_limit_ = translation_velocity_limit;
  this->rotation_velocity_limit_ = ContinuousAngle(rotation_velocity_limit);
  this->translation_acceleration_limit_ = translation_acceleration_limit;
  this->rotation_acceleration_limit_ = ContinuousAngle(rotation_acceleration_limit);
}

RobotControl::RobotControl()
  : translation_velocity_limit_(-1)
  , rotation_velocity_limit_(-1)
  , translation_acceleration_limit_(-1)
  , rotation_acceleration_limit_(-1){};

Control RobotControl::limitedControl(const Vector2d& robot_position, const ContinuousAngle& robot_orientation,
                                      const Vector2d& robot_linear_velocity,
                                      const ContinuousAngle& robot_angular_velocity) const
{
  Control res = noLimitedControl();
  if (res.angular_velocity.value() != 0.0)
  {
    double max_angular_velocity;
    if (rotation_acceleration_limit_ >= ContinuousAngle(0.0))
    {
      max_angular_velocity = robot_angular_velocity.value() + getDt() * rotation_acceleration_limit_.value();
      if (rotation_velocity_limit_ >= 0)
      {
        max_angular_velocity = std::min(rotation_velocity_limit_.value(), max_angular_velocity);
      }
    }
    else
    {
      max_angular_velocity = rotation_velocity_limit_.value();
    }
    double min_angular_velocity = -1;
    if (rotation_acceleration_limit_ >= ContinuousAngle(0.0))
    {
      min_angular_velocity =
          std::max(0.0, robot_angular_velocity.value() - getDt() * rotation_acceleration_limit_.value());
    }

    if (max_angular_velocity > 0.0)
    {
      if (res.angular_velocity.abs() >= max_angular_velocity)
      {
        assert(res.angular_velocity.value() != 0);
        res.angular_velocity *= (max_angular_velocity / (std::fabs(res.angular_velocity.value()) / security_margin));
      }
    }
    if (min_angular_velocity > 0.0)
    {
      if (res.angular_velocity.abs() < min_angular_velocity)
      {
        assert(res.angular_velocity.value() != 0);
        res.angular_velocity *= (min_angular_velocity / (std::fabs(res.angular_velocity.value()) * security_margin));
      }
    }
  }
  if (res.linear_velocity.norm() != 0)
  {
    double max_linear_velocity;
    if (translation_acceleration_limit_ >= 0.0)
    {
      max_linear_velocity = robot_linear_velocity.norm() + translation_acceleration_limit_ * getDt();
      if (translation_velocity_limit_ >= 0)
      {
        max_linear_velocity = std::min(translation_velocity_limit_, max_linear_velocity);
      }
    }
    else
    {
      max_linear_velocity = translation_velocity_limit_;
    }
    double min_linear_velocity = -1.0;
    if (translation_acceleration_limit_ >= 0.0)
    {
      min_linear_velocity =
          std::max(0.0, robot_linear_velocity.norm() - getDt() * this->translation_acceleration_limit_);
    }

    if (max_linear_velocity > 0.0)
    {
      if (res.linear_velocity.norm() >= max_linear_velocity)
      {
        assert(res.linear_velocity.norm() != 0);
        res.linear_velocity *= (max_linear_velocity / (res.linear_velocity.norm() / security_margin));
      }
    }

    if (min_linear_velocity > 0.0)
    {
      if (res.linear_velocity.norm() < min_linear_velocity)
      {
        assert(res.linear_velocity.norm() != 0);
        res.linear_velocity *= (min_linear_velocity / (res.linear_velocity.norm() * security_margin));
      }
    }
  }
  return res;
}

Control RobotControlWithPid::noLimitedControl() const
{
  return PidController::noLimitedControl();
}

double RobotControlWithPid::getDt() const
{
  return PidController::getDt();
}
