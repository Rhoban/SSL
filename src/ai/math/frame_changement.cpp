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

#include "frame_changement.h"
#include <debug.h>

FrameChangement::FrameChangement()
  : origin_(0.0, 0.0)
  , basis_(Matrix2d::identity())
  , basisChangement_(Matrix2d::identity())
  , rotation_angle_from_basis_(0.0)
{
}

void FrameChangement::setFrame(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2)
{
  assert(std::fabs(1.0 - norm(v1)) < 0.000001);
  assert(std::fabs(1.0 - norm(v2)) < 0.000001);
  assert(std::fabs(scalarProduct(v1, v2)) < 0.000001);
  origin_ = origin;
  basis_(0, 0) = v1.getX();
  basis_(0, 1) = v2.getX();
  basis_(1, 0) = v1.getY();
  basis_(1, 1) = v2.getY();
  basisChangement_ = basis_.inverse();
  rotation_angle_from_basis_ = vector2angle(v1);
}

// Convert a point in absolute coordiante to frame coordiante
rhoban_geometry::Point FrameChangement::toFrame(const rhoban_geometry::Point& point) const
{
  return vector2point(toBasis(Vector2d(point - origin_)));
}

// Convert a vector in absolute coordiante to a vector in the basis coordiante of
// the frame
Vector2d FrameChangement::toBasis(const Vector2d& vector) const
{
  return basisChangement_ * vector;
}

// Convert a point in the frame coordinate to a point in an absolute coordiante
rhoban_geometry::Point FrameChangement::fromFrame(const rhoban_geometry::Point& point) const
{
  return origin_ + fromBasis(Vector2d(point));
}

// Convert a vector in the basis of the frame coordinate to a vector in the absolute    // basis
Vector2d FrameChangement::fromBasis(const Vector2d& vector) const
{
  return basis_ * vector;
}

ContinuousAngle FrameChangement::fromFrame(const ContinuousAngle& angle) const
{
  return fromBasis(angle);
}
ContinuousAngle FrameChangement::fromBasis(const ContinuousAngle& angle) const
{
  return angle + rotation_angle_from_basis_;
}

ContinuousAngle FrameChangement::toFrame(const ContinuousAngle& angle) const
{
  return toBasis(angle);
}
ContinuousAngle FrameChangement::toBasis(const ContinuousAngle& angle) const
{
  return angle - rotation_angle_from_basis_;
}
