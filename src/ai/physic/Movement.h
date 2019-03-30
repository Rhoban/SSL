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

#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include <math/vector2d.h>
#include <iostream>
#include "MovementSample.h"

namespace RhobanSSL
{
class Movement
{
public:
  virtual Movement* clone() const = 0;

  virtual void set_sample(const MovementSample& samples) = 0;
  virtual const MovementSample& get_sample() const = 0;

  /* Return the last time of the samples */
  virtual double last_time() const = 0;

  virtual rhoban_geometry::Point linear_position(double time) const = 0;
  virtual ContinuousAngle angular_position(double time) const = 0;

  virtual Vector2d linear_velocity(double time) const = 0;
  virtual ContinuousAngle angular_velocity(double time) const = 0;

  virtual Vector2d linear_acceleration(double time) const = 0;
  virtual ContinuousAngle angular_acceleration(double time) const = 0;

  virtual void print(std::ostream& stream) const = 0;

  virtual ~Movement(){};
};

}  // namespace RhobanSSL

std::ostream& operator<<(std::ostream& stream, const RhobanSSL::Movement& movement);

#endif
