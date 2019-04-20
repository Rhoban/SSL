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

#include <physic/movement.h>

namespace rhoban_ssl
{
class MovementPredictedByIntegration : public Movement
{
private:
  MovementSample samples_;

  void check();

public:
  virtual Movement* clone() const;

  virtual void setSample(const MovementSample& samples);
  virtual const MovementSample& getSample() const;

  virtual double lastTime() const;

  virtual rhoban_geometry::Point linearPosition(double time) const;
  virtual ContinuousAngle angularPosition(double time) const;

  virtual Vector2d linearVelocity(double time) const;
  virtual ContinuousAngle angularVelocity(double time) const;

  virtual Vector2d linearAcceleration(double time) const;
  virtual ContinuousAngle angularAcceleration(double time) const;

  virtual void print(std::ostream& stream) const;

  virtual ~MovementPredictedByIntegration();
};

}  // namespace rhoban_ssl
