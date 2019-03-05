/*
    This file is part of SSL.
    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)
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

#include "follow_robot.h"
#include <math/vector2d.h>


namespace RhobanSSL {
namespace Robot_behavior {
namespace Medium {

/*
* With this behavior, the robot will go to its target until it reach a defined distance.
* In short it follow its target without collide it.
*/
FollowRobot::FollowRobot(
    Ai::AiData & ai_data, int target_id
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
    
{
    target_robot_id = target_id;
}

void FollowRobot::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    annotations.clear();
    
    rhoban_geometry::Point target_position = robot.get_movement().linear_position( ai_data.time );
    ContinuousAngle target_rotation = robot.get_movement().angular_position( ai_data.time );

    // Condition to check if the target robot is not the robot itself.
    // A robot which try to follow itself will do nothing.
    if(target_robot_id != robot.id()){

        target_position = get_robot(target_robot_id).get_movement().linear_position( ai_data.time );
        rhoban_geometry::Point robot_position = robot.get_movement().linear_position( ai_data.time );

        Vector2d vect_robot_target = target_position - robot_position;
        target_rotation = vector2angle( vect_robot_target );


       //this line "reduce" a vector :
       vect_robot_target *= (1 - TRACKING_DISTANCE/vect_robot_target.norm());;
       target_position += vector2point(vect_robot_target);
        

    }

    follower->set_following_position( target_position, target_rotation );


    follower->avoid_the_ball(false);
    follower->update(time, robot, ball);
}


void FollowRobot::set_robot_id_to_follow (int id){
    target_robot_id = id;
}

int FollowRobot::get_robot_id_to_follow() const{
    return target_robot_id;
}



Control FollowRobot::control() const {
    Control ctrl = follower->control();
    return ctrl; 
}

FollowRobot::~FollowRobot(){
    delete follower;
}

RhobanSSLAnnotation::Annotations FollowRobot::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}



}
}
}