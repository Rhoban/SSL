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

#include "consign_follower.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
ConsignFollower::ConsignFollower() : RobotBehavior()
{
}

ConsignFollower::~ConsignFollower()
{
}

void ConsignFollower::avoidTheBall(bool value)
{
}
void ConsignFollower::avoidAlly(bool value)
{
}
void ConsignFollower::avoidOpponent(bool value)
{
}
void ConsignFollower::avoidRobot(int id, bool value)
{
}

void ConsignFollower::setRadiusAvoidanceForTheBall(double radius)
{
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
