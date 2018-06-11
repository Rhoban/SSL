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

#ifndef __TANGENTS__H__
#define __TANGENTS__H__

#include <rhoban_geometry/point.h>
#include <rhoban_geometry/segment.h>
#include <vector>

namespace rhoban_geometry {

    Point center_of_cone_incircle(
        const rhoban_geometry::Point & cone_vertex,
        const rhoban_geometry::Point & cone_base_A,
        const rhoban_geometry::Point & cone_base_B,
        double circle_radius
    );    

    /*
     * Return a vector v of the 4 tangents of two circles. 
     * v[0] and v[1] are parallels tangents of the two circles
     * ( V[1] is the symetric of $V[0] beetween the two circle centers).
     * v[2] and v[3] are crossing tangents of the two circles
     * ( V[2] is the symetric of $V[3] beetween the two circle centers).
     */
    std::vector<rhoban_geometry::Segment> tangent_of_two_circle(
        const rhoban_geometry::Circle & circle_A,
        const rhoban_geometry::Circle & center_B
    );

}

#endif
