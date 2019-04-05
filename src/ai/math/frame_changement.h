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

#include "vector2d.h"
#include "continuous_angle.h"
#include <math/matrix2d.h>

class FrameChangement
{
private:
  rhoban_geometry::Point origin_;
  Matrix2d basis_;
  Matrix2d basisChangement_;
  ContinuousAngle rotation_angle_from_basis_;

public:
  FrameChangement();

  // We assume that the vector v1 and v2 are orthonormal
  void setFrame(const rhoban_geometry::Point& origin_, const Vector2d& v1, const Vector2d& v2);

  // Convert a point in absolute coordiante to frame coordiante
  rhoban_geometry::Point toFrame(const rhoban_geometry::Point& point) const;

  // Convert a vector in absolute coordiante to a vector in the basis coordiante of
  // the frame
  Vector2d toBasis(const Vector2d& vector) const;

  // Convert a point in the frame coordinate to a point in an absolute coordiante
  rhoban_geometry::Point fromFrame(const rhoban_geometry::Point& point) const;

  // Convert a vector in the basis of the frame coordinate to a vector in the absolute    // basis
  Vector2d fromBasis(const Vector2d& vector) const;

  ContinuousAngle fromFrame(const ContinuousAngle& angle) const;
  ContinuousAngle fromBasis(const ContinuousAngle& angle) const;

  ContinuousAngle toFrame(const ContinuousAngle& angle) const;
  ContinuousAngle toBasis(const ContinuousAngle& angle) const;
};
