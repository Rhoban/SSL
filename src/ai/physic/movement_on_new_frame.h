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
#include <math/frame_changement.h>

namespace rhoban_ssl
{
/*
class MovementOnNewFrame : public Movement
{
private:
  Movement* movement_;
  FrameChangement frame_;

public:
  // We assume that v1 and v2 are orthonormal
  void setFrame(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2);

  virtual Movement* clone() const;
  const Movement* getOriginalMovement() const;

  virtual double lastTime() const;

  MovementOnNewFrame(Movement* movement);

  virtual void setSample(const MovementSample<ai::Config::samples_history_size>& samples);
  virtual const MovementSample<ai::Config::samples_history_size>& getSample() const;

  virtual rhoban_geometry::Point linearPosition(double time) const;
  virtual ContinuousAngle angularPosition(double time) const;

  virtual Vector2d linearVelocity(double time) const;
  virtual ContinuousAngle angularVelocity(double time) const;

  virtual Vector2d linearAcceleration(double time) const;
  virtual ContinuousAngle angularAcceleration(double time) const;

  virtual void print(std::ostream& stream) const;

  virtual ~MovementOnNewFrame();
};
*/
}  // namespace rhoban_ssl
