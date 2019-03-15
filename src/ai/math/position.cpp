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

#include "position.h"

Position::Position() : linear(0.0, 0.0), angular(0.0)
{
}

Position::Position(const rhoban_geometry::Point& linear, const ContinuousAngle& angular)
  : linear(linear), angular(angular)
{
}

Position::Position(double x, double y, double angle) : Position(rhoban_geometry::Point(x, y), ContinuousAngle(angle))
{
}

std::ostream& operator<<(std::ostream& out, const Position& pos)
{
  out << "(lin : " << pos.linear << ", ang : " << pos.angular << ")";
  return out;
}
