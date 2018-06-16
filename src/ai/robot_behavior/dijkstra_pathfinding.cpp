/*
    This file is part of SSL.

    Copyright 2018 Rémi Fabre (remifabre1800@gmail.com)

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

#include "dijkstra_pathfinding.h"
#include <rhoban_geometry/segment.h>
#include <physic/constants.h>

namespace RhobanSSL {
namespace Robot_behavior {

Dijkstra_pathfinding::Dijkstra_pathfinding(
    Ai::AiData & ai_data, double time, double dt,
    ConsignFollower* consign_follower
):
    ConsignFollower(ai_data), 
    follower(consign_follower),
    target_position(0.0, 0.0), target_angle(0.0)
{
} 

void Dijkstra_pathfinding::compute_next_position(){
    // TODO ! !
    // TODO update graph + compute closest path and point control 
    rhoban_geometry::Point next_position; //TODO
    rhoban_geometry::Point current_robot_position = linear_position();
    
    this->follower->set_following_position( next_position, target_angle );
} 

void Dijkstra_pathfinding::set_following_position(
    const Vector2d & position_to_follow,
    const ContinuousAngle & target_angle
){
    this->target_position = position_to_follow;
    this->target_angle = target_angle;
    this->target_angle = this->robot_angular_position;
    this->target_angle.set_to_nearest(target_angle); 
    
    compute_next_position();
}

void Dijkstra_pathfinding::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );

    compute_next_position();

    follower->update( time, robot, ball );
}

Control Dijkstra_pathfinding::control() const {
    return follower->control();
}

}
}
