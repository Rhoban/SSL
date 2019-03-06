/*
    This file is part of SSL.
    
    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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

#include "prepare_strike.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {
namespace Medium {

Prepare_strike:: Prepare_strike(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) ),
    striking_point( opponent_goal_center() ) // You can change the striking_point
{
}

void  Prepare_strike::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );
    
    Vector2d ball_strickypoint_vector = striking_point - ball_position();
    ball_strickypoint_vector = ball_strickypoint_vector / ball_strickypoint_vector.norm(); // TODO: Check

    double target_radius_from_ball;

    bool is_between_strikypoint_ball;
    double x_ball = ball_position().getX();
    double x_strikypoint = striking_point.getX();
    double x_robot = robot_position.getX();

    if(x_ball > x_strikypoint) {
        is_between_strikypoint_ball = x_strikypoint < x_robot && x_robot < x_ball;
    } else {
        is_between_strikypoint_ball = x_ball < x_robot && x_robot < x_strikypoint;
    }
    
    // If the robot is between the x-axis of the ball and the x-axis of the opponent_goal_center, the scalar is lesser than to 0. 
    // If the robot is behind the x-axis of the ball, the scalar is greater than to 0.
    if(is_between_strikypoint_ball) {
        follower->avoid_the_ball(true);
        target_radius_from_ball = 0.4;
    } else {
        follower->avoid_the_ball(false);
        target_radius_from_ball = 0.3;
    }

    

    rhoban_geometry::Point target_position = ball_position() - ball_strickypoint_vector * (target_radius_from_ball);
    double target_rotation = detail::vec2angle(ball_strickypoint_vector);

    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);   

}

Control  Prepare_strike::control() const {
    Control ctrl = follower->control();
    return ctrl; 
}

 Prepare_strike::~ Prepare_strike(){
    delete follower;
}

RhobanSSLAnnotation::Annotations  Prepare_strike::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}

} // Namespace Medium
} // Namespace Robot_behavior
} // Namespace RhobanSSL
