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

#include "example_forward.h"
#include "math/vector2d.h"

namespace RhobanSSL {
namespace Robot_behavior {

#define PERIOD 10.0

ExampleForward::ExampleForward(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower(Factory::fixed_consign_follower(ai_data)),
    period(PERIOD),
    last_time(0)
{
}

void ExampleForward::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    RobotBehavior::update_time_and_position(time, robot, ball);
    if (time - last_time > period){
        follower->set_following_position(center_mark());
        last_time = time;
    }
    follower->avoid_the_ball(false);
    follower->update(time, robot, ball);
}

}
}
