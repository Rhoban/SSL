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
    // A bit weird but at this point, it's the follower that knows where to go...
    Vector2d next_point_vect;
    ContinuousAngle next_angle;
    this->follower->get_following_position(&next_point_vect, &next_angle);
    rhoban_geometry::Point next_point = vector2point(next_point_vect);
    target_position = next_point_vect;
    target_angle = next_angle;
    _count = 0;
} 

void Dijkstra_pathfinding::set_following_position(
    const Vector2d & position_to_follow,
    const ContinuousAngle & target_angle
){
    this->target_position = position_to_follow;
    this->target_angle = target_angle;
    this->target_angle = this->robot_angular_position;
    this->target_angle.set_to_nearest(target_angle); 
    // DEBUG("set_following_position at  " << position_to_follow);

    // Care, I wanted to call update_pathfinding(), but calling it without doing a 'update_time_and_position' fails. 
    // Check if this doesn't make the actual trajectory not continuous
    this->follower->set_following_position( this->target_position, target_angle );
}

// Sets the positions of the robots as obstacles and recalculates the list of obstaclepoints (=path) to get to the destination
void Dijkstra_pathfinding::update_pathfinding() {
    // There is no current way to clear the obstacles without changing ObstacleAvoider (not done yet cos shared)
    rhoban_graphs::ObstacleAvoider * oa = new rhoban_graphs::ObstacleAvoider();
    update_obstacles(oa);

    rhoban_geometry::Point current_robot_position = linear_position();
    // Actual path calculation
    DEBUG("Currrent robot : " << robot_ptr->id());
    DEBUG("findPath from " << current_robot_position << " to " << vector2point(this->target_position)); 
    
 
    float accuracy = 1.0;          
    list_of_points = oa->findPath(current_robot_position, vector2point(this->target_position), accuracy);
    
    DEBUG("(before) Nb points de passage = " << list_of_points.size());
    for (size_t i = 0; i < list_of_points.size(); i++) {
        DEBUG("(before) list_of_points[" << i << "] = " << list_of_points[i]);        
    }
    
    if (list_of_points.size() < 2) {
        // Astuce
        list_of_points.push_back(vector2point(this->target_position));
        if (list_of_points.size() < 2) {
            // Astuce
            list_of_points.push_back(vector2point(this->target_position));
        }
    }
    list_of_points.erase(list_of_points.begin() + 0);       
    
    // DEBUG("Nb points de passage = " << list_of_points.size());
/*     for (size_t i = 0; i < list_of_points.size(); i++) {
        DEBUG("list_of_points[" << i << "] = " << list_of_points[i]);        
    } */
    this->set_following_position( list_of_points[0], target_angle );  
    
    delete oa;
}

void Dijkstra_pathfinding::update_obstacles(rhoban_graphs::ObstacleAvoider * oa) {
    // Getting all the positions of the robots
    std::vector<rhoban_geometry::Point> robot_positions;
    float radius = 0.0;
    for (size_t i = 0; i < Ai::Constants::NB_OF_ROBOTS_BY_TEAM; i++) {
        const Ai::Robot & robot1 = get_robot( i,  Vision::Team::Opponent );
        if(robot1.is_present_in_vision()){
            robot_positions.push_back(robot1.get_movement().linear_position(time()));
        }
        const Ai::Robot & robot2 = get_robot( i,  Vision::Team::Ally );
        if (robot_ptr->id() == i) {
            // We're not dodging ourselves
            continue;
        }
        if(robot2.is_present_in_vision()){
            robot_positions.push_back(robot2.get_movement().linear_position(time()));
        }
    }

    radius = get_robot_radius()*1.6; // Because there is the other robot radius + ours. 
    // Adding each robot as an obstacle
    DEBUG("Nb obstacles " << robot_positions.size());            
    for (size_t i = 0; i < robot_positions.size(); i++) {
        oa->addObstacle(robot_positions[i], radius);
        DEBUG("Adding obstacle at " << robot_positions[i] << " with radius " << radius);        
    }    
    // TODO add border limits ? Penalty area ?
}

// Giving a trajectory (list of points), gives the follower a local point to follow
void Dijkstra_pathfinding::compute_next_position() {
    if (list_of_points.size() < 2) {
        return;
    }
    Vector2d next_point_vect;
    ContinuousAngle next_angle;
    this->follower->get_following_position(&next_point_vect, &next_angle);
    rhoban_geometry::Point next_point = vector2point(next_point_vect);
    
    rhoban_geometry::Point current_robot_position = linear_position();
    float dist = sqrt((next_point.getX() - current_robot_position.getX())*(next_point.getX() - current_robot_position.getX()) 
    + (next_point.getY() - current_robot_position.getY())*(next_point.getY() - current_robot_position.getY()));
    // DEBUG("**DIST = " << dist);  
    
    if (dist < _dist_for_next_point) {
        // DEBUG("Going to the next point,  list_of_points.size() = " << list_of_points.size());  
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
    Vector2d next_point_vect;
    ContinuousAngle next_angle;
    this->follower->get_following_position(&next_point_vect, &next_angle);
    // DEBUG("In dijkstra, this->get_following_position = " << next_point_vect);  
    // DEBUG("In dijkstra, this->target_position = " << this->target_position);
      
    

    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // DEBUG("Dijkstra_pathfinding::update....");        

    // Check if this is not too time consumming...
    if (robot_ptr->id() == 4) {
        _count++;
        if (_count > 50) {
            _count = 0;
            update_pathfinding();  
        }
        compute_next_position();
    }
    // DEBUG("I'm currently at " << linear_position());            

    follower->update( time, robot, ball );
    // DEBUG("Follower updated... ");               
}

Control Dijkstra_pathfinding::control() const {
    return follower->control();
}

Dijkstra_pathfinding::~Dijkstra_pathfinding(){
    delete follower;
}

}
}
