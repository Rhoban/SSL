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

#include "movement_sample.h"
#include <assert.h>
#include <debug.h>

using namespace rhoban_geometry;

namespace RhobanSSL
{
PositionSample::PositionSample() : PositionSample(0.0, Point(0.0, 0.0), ContinuousAngle(0.0))
{
}

PositionSample::PositionSample(double time, const Point& linear_position, const ContinuousAngle& angular_position)
  : time(time), linear_position(linear_position), angular_position(angular_position)
{
}

void MovementSample::insert(const PositionSample& sample)
{
  assert(sample.time >= (*this)[0].time);
  // if( sample.time == (*this)[0].time ){
  if (fabs(sample.time - (*this)[0].time) < 0.01)
  {
    (*this)[0] = sample;

    double filtered_dt = 0.0;
    // small filter
    for (int it = 0; it < (this->size() - 2); it++)
    {
      filtered_dt += ((*this)[it].time - (*this)[it + 1].time);
    }
    this->dts[0] = filtered_dt / (this->size() - 2);
  }
  else
  {
    circular_vector<PositionSample>::insert(sample);
    double filtered_dt = 0.0;
    // small filter
    for (int it = 0; it < (this->size() - 2); it++)
    {
      filtered_dt += ((*this)[it].time - (*this)[it + 1].time);
    }
    this->dts.insert(filtered_dt / (this->size() - 2));
  }
}

bool MovementSample::is_valid() const
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

MovementSample::MovementSample(unsigned int size, double default_dt) : circular_vector<PositionSample>(size), dts(size)
{
  for (unsigned int i = 0; i < size; i++)
  {
    dts[i] = default_dt;
  }
}

MovementSample::MovementSample() : circular_vector<PositionSample>(), dts()
{
}

double MovementSample::time(unsigned int i) const
{
  return (*this)[i].time;
}

double MovementSample::dt(unsigned int i) const
{
  return this->dts[i];
}

Point MovementSample::linear_position(unsigned int i) const
{
  return (*this)[i].linear_position;
}

ContinuousAngle MovementSample::angular_position(unsigned int i) const
{
  return (*this)[i].angular_position;
}

Vector2d MovementSample::linear_velocity(unsigned int i) const
{
  return (linear_position(i) - linear_position(i + 1)) / dt(i);
}

ContinuousAngle MovementSample::angular_velocity(unsigned int i) const
{
  // TODO Check this
  return (angular_position(i) - angular_position(i + 1)) / dt(i);
}

Vector2d MovementSample::linear_acceleration(unsigned int i) const
{
  return (linear_velocity(i) - linear_velocity(i + 1)) / dt(i);
}

ContinuousAngle MovementSample::angular_acceleration(unsigned int i) const
{
  return (angular_velocity(i) - angular_velocity(i + 1)) / dt(i);
}

}  // namespace RhobanSSL

std::ostream& operator<<(std::ostream& stream, const RhobanSSL::PositionSample& pos)
{
  stream << "("
            "t="
         << pos.time << ", "
                        "lin="
         << pos.linear_position << ", "
                                   "ang="
         << pos.angular_position << ")";
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const RhobanSSL::MovementSample& mov)
{
  stream << static_cast<circular_vector<RhobanSSL::PositionSample>>(mov);
  return stream;
}
