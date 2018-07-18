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

#include "robot_follower.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <math/ContinuousAngle.h>

namespace RhobanSSL {
namespace Robot_behavior {


RobotFollower::RobotFollower(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    robot_to_follow_id(-1),
    team(Vision::Team::Ally),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void RobotFollower::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Now
    //  this->robot_linear_position
    //  this->robot_angular_position
    // are all avalaible


    const Ai::Robot & robot_to_follow = get_robot( robot_to_follow_id, robot_to_follow_team );
    rhoban_geometry::Point position = robot_to_follow.get_movement().linear_position( time );
    rhoban_geometry::Point target_position = position + translation;

    rhoban_geometry::Point robot_position = robot.get_movement().linear_position( time );
    double target_rotation = detail::vec2angle(ball_position() - robot_position);

    follower->avoid_the_ball(true);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control RobotFollower::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler !
    return ctrl;
}

RobotFollower::~RobotFollower(){
    delete follower;
}


void RobotFollower::declare_robot_to_follow( int robot_id, const Vector2d & translation, Vision::Team team ){
    robot_to_follow_id = robot_id;
    this->translation = translation;
    robot_to_follow_team = team;
}


}
}
