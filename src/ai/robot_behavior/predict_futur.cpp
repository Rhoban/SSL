/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#include "predict_futur.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


PredictFutur::PredictFutur(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    use_custom_vector(false),
    striking_point( opponent_goal_center() ),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void PredictFutur::update(
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

    annotations.clear();
    
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( time );
    const rhoban_geometry::Point & robot_position_futur = robot.get_movement().linear_position( time + 0.5 );
    const rhoban_geometry::Point & ball_position_futur = ball.get_movement().linear_position( time + 0.5 );
    annotations.addCross( robot_position_futur.x, robot_position_futur.y, "blue");
    annotations.addCross( ball_position_futur.x, ball_position_futur.y, "red");

    //rhoban_geometry::Point oponent_goal_point = oponent_goal_center();
    //rhoban_geometry::Point left_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0, ai_data.field.goalWidth / 2.0 );
    //rhoban_geometry::Point right_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0 );

    Vector2d ball_goal_vector = striking_point - ball_position();
    Vector2d ball_robot_vector = robot_position - ball_position();
    //Vector2d ball_l_post_vector = left_post_position - ball_position();
    //Vector2d ball_r_post_vector = right_post_position - ball_position();

    ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
    ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();
    //ball_l_post_vector = ball_l_post_vector / ball_l_post_vector.norm();
    //ball_r_post_vector = ball_r_post_vector / ball_r_post_vector.norm();


    //double goal_visible_angle = scalar_product( ball_l_post_vector , ball_r_post_vector );

    double target_radius_from_ball;
    double scalar_ball_robot = - scalar_product( ball_robot_vector , ball_goal_vector );


    if ( scalar_ball_robot < 0 ) {
        follower->avoid_the_ball(true);
        target_radius_from_ball = 1.5;
    } else {
        follower->avoid_the_ball(false);
        //target_radius_from_ball = 1 / ( 2*(scalar_ball_robot - 1.2) ) + 2;
        target_radius_from_ball = 1.0 / ( 4.0*(scalar_ball_robot - 1.2) ) + 1.0;

        //target_radius_from_ball = 1.0 /(2.0 *(scalar_ball_robot - (goal_visible_angle + 0.21))) + 1.0 / (goal_visible_angle + 0.21) + 2.0 * goal_visible_angle - 0.8;

        //if ( scalar_ball_robot < goal_visible_angle) {
        //    target_radius_from_ball = 1.0;
        //} else {
        //    target_radius_from_ball = -0.3;
        //}
        //
    }

    rhoban_geometry::Point target_position = ball_position() - ball_goal_vector * target_radius_from_ball;
    double target_rotation = detail::vec2angle(ball_goal_vector);

    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control PredictFutur::control() const {
    Control ctrl = follower->control();
    ctrl.charge = true;
    ctrl.kick = true;
    return ctrl;
}


void PredictFutur::declare_point_to_strik( rhoban_geometry::Point point ){
    striking_point = point;
}


PredictFutur::~PredictFutur(){
    delete follower;
}

RhobanSSLAnnotation::Annotations PredictFutur::get_annotations() const {
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations( this->annotations );
  annotations.addAnnotations( follower->get_annotations() );
  return annotations;;
}

}
}
