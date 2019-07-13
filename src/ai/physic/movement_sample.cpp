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

namespace rhoban_ssl
{
PositionSample::PositionSample() : PositionSample(0.0, Point(0.0, 0.0), ContinuousAngle(0.0))
{
}

PositionSample::PositionSample(double time, const Point& linear_position, const ContinuousAngle& angular_position)
  : time(time), linear_position(linear_position), angular_position(angular_position)
{
}

}  // namespace rhoban_ssl

std::ostream& operator<<(std::ostream& stream, const rhoban_ssl::PositionSample& pos)
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
