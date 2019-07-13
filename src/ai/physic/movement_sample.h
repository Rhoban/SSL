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

#include <rhoban_geometry/point.h>
#include <math/continuous_angle.h>
#include <math/vector2d.h>
#include <math/circular_vector.h>
namespace rhoban_ssl
{
struct PositionSample
{
  double time;
  rhoban_geometry::Point linear_position;
  ContinuousAngle angular_position;

  PositionSample();
  PositionSample(double time, const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position);
};

template <int N>
struct MovementSample : public CircularVector<PositionSample, N>
{
  CircularVector<double, N> dts;

  MovementSample(double default_dt = 1.0 / 60.0)
  {
    for (unsigned int i = 0; i < N; i++)
    {
      dts[i] = default_dt;
    }
  }
  // MovementSample();

  double time(unsigned int i = 0) const
  {
    return (*this)[i].time;
  }
  double dt(unsigned int i = 0) const
  {
    return this->dts[i];
  }

  rhoban_geometry::Point linearPosition(unsigned int i = 0) const
  {
    return (*this)[i].linear_position;
  }
  ContinuousAngle angularPosition(unsigned int i = 0) const
  {
    return (*this)[i].angular_position;
  }

  Vector2d linearVelocity(unsigned int i = 0) const
  {
    return (linearPosition(i) - linearPosition(i + 1)) / dt(i);
  }
  ContinuousAngle angularVelocity(unsigned int i = 0) const
  {
    return (angularPosition(i) - angularPosition(i + 1)) / dt(i);
  }

  Vector2d linearAcceleration(unsigned int i = 0) const
  {
    return (linearVelocity(i) - linearVelocity(i + 1)) / dt(i);
  }
  ContinuousAngle angularAcceleration(unsigned int i = 0) const
  {
    return (angularVelocity(i) - angularVelocity(i + 1)) / dt(i);
  }

  bool isValid() const
  {
    assert(this->size() >= 1);
    for (unsigned int i = 0; i < this->size() - 1; i++)
    {
      if ((*this)[i].time <= (*this)[i + 1].time)
      {
        return false;
      }
    }
    return true;
  }
  void insert(const PositionSample& sample)
  {
    assert(sample.time >= (*this)[0].time);
    // if( sample.time == (*this)[0].time ){
    if (fabs(sample.time - (*this)[0].time) < 0.01)
    {
      (*this)[0] = sample;

      double filtered_dt = 0.0;
      // small filter
      for (uint it = 0; it < (this->size() - 2); it++)
      {
        filtered_dt += ((*this)[it].time - (*this)[it + 1].time);
      }
      this->dts[0] = filtered_dt / (this->size() - 2);
    }
    else
    {
      CircularVector<PositionSample, N>::insert(sample);
      double filtered_dt = 0.0;
      // small filter
      for (uint it = 0; it < (this->size() - 2); it++)
      {
        filtered_dt += ((*this)[it].time - (*this)[it + 1].time);
      }
      this->dts.insert(filtered_dt / (this->size() - 2));
    }
  }
};

}  // namespace rhoban_ssl

std::ostream& operator<<(std::ostream& stream, const rhoban_ssl::PositionSample& pos);

template <int N>
std::ostream& operator<<(std::ostream& stream, const rhoban_ssl::MovementSample<N>& mov)
{
  stream << static_cast<CircularVector<rhoban_ssl::PositionSample, N>>(mov);
  return stream;
}
