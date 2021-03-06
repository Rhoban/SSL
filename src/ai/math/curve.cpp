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

#include <assert.h>
#include "curve.h"
#include <debug.h>

DifferentiableVelocityConsign::DifferentiableVelocityConsign(double distance, double max_velocity,
                                                             double max_acceleration)
  : distance(distance), max_velocity(max_velocity), max_acceleration(max_acceleration)
{
  assert(max_acceleration > 0.0);  // Acceleration should be Greater than 0.
  if (not(distance >= (2 * max_velocity * max_velocity / max_acceleration)))
  {
    std::cerr << "Warning : Distance curve is too short for given maximal velocity and acceleration." << std::endl;
    std::cerr << "Warning :    we should have : distance > 2*max_velocity*max_velocity/max_acceleration" << std::endl;
    double old_velocity = max_velocity;
    max_velocity = std::sqrt(distance * max_acceleration / 2.0);
    std::cerr << "Warning :    we reduce the velocity from " << old_velocity << " to " << max_velocity << std::endl;
  }
}

double DifferentiableVelocityConsign::operator()(double t)
{
  double x = time_of_acceleration();
  double a = max_acceleration;
  double tm = time_of_deplacement();
  if (t <= 0)
    return 0;
  if (t <= x / 2.0)
    return a * t * t / x;
  if (t <= x)
    return a * x / 2 - a * (t - x) * (t - x) / x;
  if (t <= tm - x)
    return a * x / 2;
  if (t <= tm - x / 2.0)
    return a * x / 2 - a * (t - tm + x) * (t - tm + x) / x;
  if (t <= tm)
    return a * (t - tm) * (t - tm) / x;
  return 0.0;
}
double DifferentiableVelocityConsign::time_of_deplacement()
{
  double x = time_of_acceleration();
  double a = max_acceleration;
  return 2 * distance / (a * x) + x;
}
double DifferentiableVelocityConsign::time_of_acceleration()
{
  return 2 * max_velocity / max_acceleration;
}

ContinuousVelocityConsign::ContinuousVelocityConsign(double distance, double max_velocity, double max_acceleration)
  : distance(distance), max_velocity(max_velocity), max_acceleration(max_acceleration)
{
  assert(max_acceleration > 0.0);  // Acceleration should be Greater than 0.
}

double ContinuousVelocityConsign::operator()(double t)
{
  double x = time_of_acceleration();
  double a = max_acceleration;
  double tm = time_of_deplacement();
  assert(a > 0);
  assert(tm > 0);
  assert(x > 0);
  if (t <= 0)
    return 0;
  if (t <= x)
    return a * t;
  if (t <= tm - x)
    return a * x;
  if (t <= tm)
    return -a * (t - tm);
  return 0.0;
}
double ContinuousVelocityConsign::time_of_deplacement()
{
  double x = time_of_acceleration();
  double a = max_acceleration;
  assert(a > 0);
  assert(x > 0);
  return distance / (a * x) + x;
}
double ContinuousVelocityConsign::time_of_acceleration()
{
  return max_velocity / max_acceleration;
}

void Curve2d::init()
{
  if (curve_length < 0)
  {
    this->curve_length = lengthIterator().next(1.0);
  }
}

Curve2d::Curve2d(const std::function<Vector2d(double u)>& curve, double step_curve_parameter)
  : curve(curve), step_curve_parameter(step_curve_parameter), curve_length(-1.0)
{
  init();
};

Curve2d::Curve2d(const std::function<Vector2d(double u)>& curve, double step_curve_parameter, double curve_length)
  : curve(curve), step_curve_parameter(step_curve_parameter), curve_length(curve_length)
{
  init();
};

Curve2d::Curve2d(const Curve2d& curve)
  : curve(curve.curve), step_curve_parameter(curve.step_curve_parameter), curve_length(curve.curve_length)
{
  init();
};

Vector2d Curve2d::operator()(double u) const
{
  return this->curve(u);
}

double Curve2d::size() const
{
  return this->curve_length;
}

double Curve2d::arcLength(double u) const
{
  return Length(*this)(u);
}

double Curve2d::inverseOfArcLength(double l) const
{
  return InverseOfLength(*this)(l);
}

void RenormalizedCurve::init()
{
  this->time_max = time(this->curve_length);
}

RenormalizedCurve::RenormalizedCurve(const std::function<Vector2d(double u)>& curve, double step_curve_parameter,
                                     const std::function<double(double t)>& velocity_consign, double step_time,
                                     double length_tolerance)
  : Curve2d(curve, step_curve_parameter)
  , velocity_consign(velocity_consign)
  , length_tolerance(length_tolerance)
  , step_time(step_time)
{
  init();
};

RenormalizedCurve::RenormalizedCurve(const Curve2d& curve, const std::function<double(double t)>& velocity_consign,
                                     double step_time, double length_tolerance)
  : Curve2d(curve), velocity_consign(velocity_consign), length_tolerance(length_tolerance), step_time(step_time)
{
  init();
};

double RenormalizedCurve::maxTime() const
{
  return this->time_max;
}

Vector2d RenormalizedCurve::originalCurve(double u) const
{
  return curve(u);
}

void RenormalizedCurve::setStepTime(double dt)
{
  assert(dt > 0.0);
  this->step_time = dt;
  init();
}

double RenormalizedCurve::getStepTime() const
{
  return this->step_time;
}

double RenormalizedCurve::positionConsign(double t) const
{
  return PositionConsign(*this)(t);
}

double RenormalizedCurve::errorPositionConsign() const
{
  double max_velocity = 0;
  for (double t = 0; t < maxTime(); t += this->step_time)
  {
    max_velocity = std::max(max_velocity, velocity_consign(t));
  }
  return this->step_time * max_velocity;
}

RenormalizedCurve::TimeCurve RenormalizedCurve::timeIterator() const
{
  return TimeCurve(*this);
}
double RenormalizedCurve::time(double length) const
{
  return TimeCurve(*this)(length);
}

Vector2d RenormalizedCurve::operator()(double t) const
{
  //    return original_curve( this->inverse_of_arc_length( position_consign(t) ) );
  return CurveIterator(*this)(t);
  // original_curve( this->inverse_of_arc_length( position_consign(t) ) );
}
