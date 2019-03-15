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

#ifndef __PHYSIC__COLLISION__H__
#define __PHYSIC__COLLISION__H__

#include "Movement.h"

namespace RhobanSSL
{
/*
 * This function computes the future time of collision of two movement with respect to
 * the current time `time`.
 * Let res be the result of this function. If, in the future, there is no collision then
 * res.first is set to false. Suppose now that we have a collision in the futre at t_m (t<timeà. Then, than res.second
 * is equal to t_m.
 */
std::pair<bool, double> collision_time(double radius_1, const Movement& movement_1, double radius_2,
                                       const Movement& movement_2, double radius_error, double time);
std::pair<bool, double> collision_time(double radius_A, const rhoban_geometry::Point& A, const Vector2d& V_A,
                                       double radius_B, const rhoban_geometry::Point& B, const Vector2d& V_B,
                                       double radius_error);

};  // namespace RhobanSSL

#endif
