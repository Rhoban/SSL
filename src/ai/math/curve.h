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

#include <debug.h>
#include <vector>
#include <functional>
#include <math/vector2d.h>

struct ContinuousVelocityConsign
{
  double distance;
  double max_velocity;
  double max_acceleration;

  ContinuousVelocityConsign(double distance, double max_velocity, double max_acceleration);

  double operator()(double t);
  double time_of_deplacement();
  double time_of_acceleration();
};

struct DifferentiableVelocityConsign
{
  double distance;
  double max_velocity;
  double max_acceleration;

  DifferentiableVelocityConsign(double distance, double max_velocity, double max_acceleration);

  double operator()(double t);
  double time_of_deplacement();
  double time_of_acceleration();
};

class Curve2d;

class Curve2d
{
public:
  std::function<Vector2d(double u)> curve;

  double step_curve_parameter;
  double curve_length;

  void init();

public:
  struct Length
  {
    const Curve2d& _this;
    double v;
    double length;
    Vector2d old;

    Length(const Curve2d& _this) : _this(_this), v(0), length(0), old(_this.curve(0.0))
    {
    }

    double next(double u)
    {
      assert(v <= u);
      for (; v <= u; v += _this.step_curve_parameter)
      {
        Vector2d current = _this.curve(v);
        length += norm_2(current - old);
        old = current;
      }
      return length + norm_2(_this.curve(u) - old);
    }

    double operator()(double u)
    {
      if (u <= 0)
        return 0.0;
      if (u >= 1.0)
        return _this.curve_length;
      return next(u);
    }
  };

  struct InverseOfLength
  {
    const Curve2d& _this;
    double u;
    double length;
    Vector2d old;

    InverseOfLength(const Curve2d& _this) : _this(_this), u(0), length(0), old(_this.curve(0.0))
    {
    }

    double operator()(double l)
    {
      if (l <= 0)
        return 0.0;
      if (l >= _this.curve_length)
        return 1.0;
      for (; length < l; u += _this.step_curve_parameter)
      {
        Vector2d current = _this.curve(u);
        length += norm_2(current - old);
        old = current;
      }
      return u;
    }
  };

public:
  Curve2d(const std::function<Vector2d(double u)>& curve, double step_curve_parameter);
  Curve2d(const std::function<Vector2d(double u)>& curve, double step_curve_parameter, double curve_length);
  Curve2d(const Curve2d& curve);

  Vector2d operator()(double u) const;

  double arcLength(double u) const;
  Length lengthIterator() const
  {
    return Length(*this);
  }

  double inverseOfArcLength(double l) const;
  InverseOfLength InverseOfLengthIterator() const
  {
    return InverseOfLength(*this);
  }

  double size() const;
};

class RenormalizedCurve;

class RenormalizedCurve : public Curve2d
{
public:
  std::function<double(double t)> velocity_consign;
  double time_max;

  double length_tolerance;
  double step_time;

  void init();

public:
  struct PositionConsign
  {
    const RenormalizedCurve& _this;
    double pos;
    double u;

    PositionConsign(const RenormalizedCurve& _this) : _this(_this), pos(0.0), u(0.0)
    {
    }

    double operator()(double t)
    {
      if (u >= t)
        return pos;
      for (; u < t; u += _this.step_curve_parameter)
      {
        pos += (_this.step_curve_parameter * _this.velocity_consign(u));
      }
      return pos;
    }
  };

  struct TimeCurve
  {
    const RenormalizedCurve& _this;
    double res;
    double t;

    TimeCurve(const RenormalizedCurve& _this) : _this(_this), res(0.0), t(0.0)
    {
    }

    double operator()(double length)
    {
      assert(0 <= length);
      assert(length <= _this.curve_length);

      for (; res < length - _this.length_tolerance; t += _this.step_time)
      {
        assert(_this.velocity_consign(t) >= 0.0);
        res += _this.step_time * _this.velocity_consign(t);
      }
      return t;
    }
  };

  RenormalizedCurve(const std::function<Vector2d(double u)>& curve, double step_curve_parameter,
                    const std::function<double(double t)>& velocity_consign, double step_time, double length_tolerance);
  RenormalizedCurve(const Curve2d& curve, const std::function<double(double t)>& velocity_consign, double step_time,
                    double length_tolerance);

  void setStepTime(double dt);

  double maxTime() const;
  Vector2d originalCurve(double u) const;
  double time(double length) const;
  TimeCurve timeIterator() const;
  double positionConsign(double t) const;

  struct CurveIterator
  {
    const RenormalizedCurve& _this;
    InverseOfLength inverse_length_iterator;
    PositionConsign position_consign_iterator;

    CurveIterator(const RenormalizedCurve& _this)
      : _this(_this)
      , inverse_length_iterator(_this.InverseOfLengthIterator())
      , position_consign_iterator(PositionConsign(_this))
    {
    }

    Vector2d operator()(double t)
    {
      if (_this.time_max <= t)
      {
        return _this.originalCurve(1.0);
      }
      return _this.originalCurve(inverse_length_iterator(position_consign_iterator(t)));
    }
  };

  CurveIterator curveIterator() const
  {
    return CurveIterator(*this);
  }
  PositionConsign positionConsignIterator() const
  {
    return PositionConsign(*this);
  }
  Vector2d operator()(double t) const;

  double getTimeMax() const
  {
    return time_max;
  };
  double errorPositionConsign() const;
  double getStepTime() const;
};
