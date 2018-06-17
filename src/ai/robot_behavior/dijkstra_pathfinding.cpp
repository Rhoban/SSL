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
#include <debug.h>



namespace RhobanSSL {
namespace Robot_behavior {

Dijkstra_pathfinding::Dijkstra_pathfinding(
    Ai::AiData & ai_data, double time, double dt,
    ConsignFollower* consign_follower, float dist_for_next_point
):
    ConsignFollower(ai_data), 
    follower(consign_follower),
    target_position(0.0, 0.0), target_angle(0.0), _dist_for_next_point(dist_for_next_point)
{
} 

void Dijkstra_pathfinding::set_following_position(
    const Vector2d & position_to_follow,
    const ContinuousAngle & target_angle
){
    this->target_position = position_to_follow;
    this->target_angle = target_angle;
    this->target_angle = this->robot_angular_position;
    this->target_angle.set_to_nearest(target_angle); 
    DEBUG("set_following_position at  " << position_to_follow);

    // Care, I wanted to call update_pathfinding(), but calling it without doing a 'update_time_and_position' fails. 
    // Check if this doesn't make the actual trajectory not continuous
    this->follower->set_following_position( this->target_position, target_angle );
}

// Sets the positions of the robots as obstacles and recalculates the list of pobstacleoints (=path) to get to the destination
void Dijkstra_pathfinding::update_pathfinding() {
    // There is no current way to clear the obstacles without changing ObstacleAvoider (not done yet cos shared)
    rhoban_graphs::ObstacleAvoider * oa = new rhoban_graphs::ObstacleAvoider();
    update_obstacles(oa);

    rhoban_geometry::Point current_robot_position = linear_position();
    // Actual path calculation
    list_of_points = oa->findPath(current_robot_position, vector2point(this->target_position));
    if (list_of_points.size() < 1) {
        // Astuce
        list_of_points.push_back(current_robot_position);
    }
    this->follower->set_following_position( list_of_points[0], target_angle );  
    
    delete oa;
}

void Dijkstra_pathfinding::update_obstacles(rhoban_graphs::ObstacleAvoider * oa) {
    // Getting all the positions of the robots
    std::vector<rhoban_geometry::Point> robot_positions;
    float radius = 0.0;
    Ai::Robot robot;
    for (size_t i = 0; i <= 7; i++) {
        robot = get_robot( i,  Vision::Team::Opponent );
        if(robot.is_present_in_vision()){
            robot_positions.push_back(robot.get_movement().linear_position(time()));
        }
        robot = get_robot( i,  Vision::Team::Ally );
        if(robot.is_present_in_vision()){
            robot_positions.push_back(robot.get_movement().linear_position(time()));
        }
    }

    radius = get_robot_radius(); 
    // Adding each robot as an obstacle
    for (size_t i = 0; i < robot_positions.size(); i++) {
        oa->addObstacle(robot_positions[i], radius);
        DEBUG("Adding obstacle at " << robot_positions[i]);        
    }
}

// Giving a trajectory (list of points), gives the follower a local point to follow
void Dijkstra_pathfinding::compute_next_position() {
    if (list_of_points.size() < 2) {
        return;
    }
    rhoban_geometry::Point next_point = list_of_points[0];
    rhoban_geometry::Point current_robot_position = linear_position();
    float dist = sqrt((next_point.getX() - current_robot_position.getX())*(next_point.getX() - current_robot_position.getX()) 
    + (next_point.getY() - current_robot_position.getY())*(next_point.getY() - current_robot_position.getY()));
    
    if (dist < _dist_for_next_point) {
        DEBUG("Going to the next point,  list_of_points.size() = " << list_of_points.size());  
        // The follower will follow this point now
        this->follower->set_following_position( list_of_points[0], target_angle );  
        list_of_points.erase(list_of_points.begin() + 0);        
    }
}

void Dijkstra_pathfinding::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );

    // Check if this is not too time consumming...
    update_pathfinding();
    compute_next_position();

    follower->update( time, robot, ball );
}

Control Dijkstra_pathfinding::control() const {
    return follower->control();
}

Dijkstra_pathfinding::~Dijkstra_pathfinding(){
    delete follower;
}

}
}
