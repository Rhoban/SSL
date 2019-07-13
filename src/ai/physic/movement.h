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

#include <math/vector2d.h>
#include <iostream>
#include "movement_sample.h"
#include <config.h>

namespace rhoban_ssl
{
class Movement
{
public:
  virtual Movement* clone() const = 0;

  virtual void setSample(const MovementSample<ai::Config::samples_history_size>& samples) = 0;
  virtual const MovementSample<ai::Config::samples_history_size>& getSample() const = 0;

  /* Return the last time of the samples */
  virtual double lastTime() const = 0;

  virtual rhoban_geometry::Point linearPosition(double time) const = 0;
  virtual ContinuousAngle angularPosition(double time) const = 0;

  virtual Vector2d linearVelocity(double time) const = 0;
  virtual ContinuousAngle angularVelocity(double time) const = 0;

  virtual Vector2d linearAcceleration(double time) const = 0;
  virtual ContinuousAngle angularAcceleration(double time) const = 0;

  virtual void print(std::ostream& stream) const = 0;

  virtual ~Movement();
};

}  // namespace rhoban_ssl

std::ostream& operator<<(std::ostream& stream, const rhoban_ssl::Movement& movement);
