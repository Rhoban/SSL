/*
    This file is part of SSL.
    
    Copyright 2018 Schmitz Etienne (hello@etienne-schmitz.com)

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

#include "begginer_striker.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


Begginer_striker::Begginer_striker(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    striking_point( oponent_goal_center() ),       
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Begginer_striker::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );

    Vector2d ball_goal_vector = striking_point - ball_position();
    Vector2d ball_robot_vector = robot_position - ball_position();
    double dist_ball_robot = ball_robot_vector.norm();

    ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
    ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();


    double target_radius_from_ball;
    double scalar_ball_robot = - scalar_product( ball_robot_vector , ball_goal_vector );
    

    if ( scalar_ball_robot < 0 ) {
        follower->avoid_the_ball(true);
        target_radius_from_ball = 0.4;
    } else {
        follower->avoid_the_ball(false);
        target_radius_from_ball = 1.0 / ( 24.0 * (scalar_ball_robot - 1.04) ) + 0.44;

        if ( dist_ball_robot < 0.4 ) {
            follower->avoid_opponent(false);
        } 
    }
    if (dist_ball_robot > 0.4) {
        follower->avoid_opponent(true);
    }

    rhoban_geometry::Point target_position = ball_position() - ball_goal_vector * (target_radius_from_ball);
    double target_rotation = detail::vec2angle(ball_goal_vector);

    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);   
}

Control Begginer_striker::control() const {
    Control ctrl = follower->control();
    ctrl.charge = true;
    ctrl.kick = true; 
    return ctrl; 
}


Begginer_striker::~Begginer_striker(){
    delete follower;
}

RhobanSSLAnnotation::Annotations Begginer_striker::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}

}
}
