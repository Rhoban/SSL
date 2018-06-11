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

#include "robot_control_with_position_following.h"

void RobotControlWithPositionFollowing::set_goal(
    const Vector2d & position, ContinuousAngle orientation
){
    this->position = position;
    this->orientation = orientation;
    set_static(false);
}

ContinuousAngle RobotControlWithPositionFollowing::goal_orientation( double t ) const {
    return orientation;
}

Vector2d RobotControlWithPositionFollowing::goal_position( double t ) const {
    return position;
}

