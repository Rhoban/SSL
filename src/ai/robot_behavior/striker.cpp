/*
    This file is part of SSL.

    Copyright 2018 TO COMPLETE 

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

#include "striker.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>

namespace RhobanSSL {
namespace Robot_behavior {


Striker::Striker(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Striker::update(
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
    
    //int robot_id = 2;
    //const Robots_table & robot_table = ai_data.robots.at(Vision::Team::Ally);
    //const Ai::Robot & robot = robot_table.at(robot_id);
    
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );
    
    rhoban_geometry::Point oponent_goal_point = oponent_goal_center();
    //rhoban_geometry::Point left_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0, ai_data.field.goalWidth / 2.0 );
    //rhoban_geometry::Point right_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0 );

    Vector2d ball_goal_vector = oponent_goal_point - ball_position();
    Vector2d ball_robot_vector = robot_position - ball_position();
    //Vector2d ball_l_post_vector = left_post_position - ball_position();
    //Vector2d ball_r_post_vector = right_post_position - ball_position();

    ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
    ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();
    //ball_l_post_vector = ball_l_post_vector / ball_l_post_vector.norm();
    //ball_r_post_vector = ball_r_post_vector / ball_r_post_vector.norm();



    //double cos_60 = 0.5000;
    //double cos_45 = 0.7071;
    //double cos_25 = 0.9006;
    //double cos_15 = 0.9659;
    //double cos_10 = 0.9848;
    //double cos_5  = 0.9961;



    //double goal_visible_angle = scalar_product( ball_l_post_vector , ball_r_post_vector );

    double target_radius_from_ball;
    double scalar_ball_robot = - scalar_product( ball_robot_vector , ball_goal_vector );

    if ( scalar_ball_robot < 0 ) {
        follower->avoid_the_ball(true);
        target_radius_from_ball = 1.5;
    } else {
        follower->avoid_the_ball(false);
        target_radius_from_ball = 1 / ( 2*(scalar_ball_robot - 1.2) ) + 2;
        //target_radius_from_ball = 1.0 /(2.0 *(scalar_ball_robot - (goal_visible_angle + 0.21))) + 1.0 / (goal_visible_angle + 0.21) + 2.0 * goal_visible_angle - 0.8;

        //if ( scalar_ball_robot < goal_visible_angle) {
        //    target_radius_from_ball = 1.0;
        //} else {
        //    target_radius_from_ball = -0.3;
        //}
        //
    }



    Vector2d target_position = Vector2d(ball_position()) - ball_goal_vector * (target_radius_from_ball);
    double target_rotation = detail::vec2angle(ball_goal_vector);

    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);   
}

Control Striker::control() const {
    Control ctrl = follower->control();
    ctrl.charge = true;
    ctrl.kick = true;
    // ctrl.spin = true; // We active the dribler ! 
    return ctrl; 
}

Striker::~Striker(){
    delete follower;
}

RhobanSSLAnnotation::Annotations Striker::get_annotations() const {
    return follower->get_annotations();
}

}
}


//Trash ZONE



//    if ( robot_ball_vector.norm() > 0.5 ) {
//        target_radius_from_ball = 0.1;
//    } else {
//        target_radius_from_ball = -0.1;
//    }



    //rhoban_geometry::Point waiting_position = rhoban_geometry::Point(0.0, 0.0) + oponent_goal_point / 2 ;
    //double strik_rotation = std::atan2( -direction.getY(), -direction.getX() );
    //double robot_toward_ball = std::atan2( robot_ball_vector.getY(), robot_ball_vector.getX() );


//if ( scalar_product( robot_position - Vector2d(ball_position()), target_position - Vector2d(ball_position())) > 0 ) {
    


//    if ( ball_position.getX() > 0 ) {
//        follower->set_following_position(target_position, target_rotation);
//    } else {
//        follower->set_following_position(waiting_position, target_rotation);
//    }




//    if ( std::abs(strik_rotation - robot_toward_ball) > (M_PI) ) {
//        follower->avoid_the_ball(true);
//        target_radius_from_ball = 1.5;
//    } else {
//        follower->avoid_the_ball(false);
        //target_radius_from_ball = 1 / (1 + std::pow(2,std::abs((strik_rotation - robot_toward_ball) / M_PI - 2 ))) * 3 - 0.8;

        //if ( std::abs(strik_rotation - robot_toward_ball) > (M_PI / 4) ) {
//            target_radius_from_ball = 0.5;
        //} else {
        //    target_radius_from_ball = -0.3;
        //}
//    }




