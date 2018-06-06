#include "defensor.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


Defensor::Defensor(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Defensor::update(
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
    rhoban_geometry::Point left_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0, ai_data.field.goalWidth / 2.0 );
    rhoban_geometry::Point right_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0 );

    Vector2d ball_goal_vector = oponent_goal_point - ball_position();
    Vector2d ball_robot_vector = robot_position - ball_position();
    Vector2d ball_l_post_vector = left_post_position - ball_position();
    Vector2d ball_r_post_vector = right_post_position - ball_position();

    ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
    ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();
    ball_l_post_vector = ball_l_post_vector / ball_l_post_vector.norm();
    ball_r_post_vector = ball_r_post_vector / ball_r_post_vector.norm();



    //double cos_60 = 0.5000;
    //double cos_45 = 0.7071;
    //double cos_25 = 0.9006;
    //double cos_15 = 0.9659;
    //double cos_10 = 0.9848;
    //double cos_5  = 0.9961;



    double goal_visible_angle = scalar_product( ball_l_post_vector , ball_r_post_vector );

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
    double target_rotation = std::atan2( -ball_goal_vector.getY(), -ball_goal_vector.getX() );

    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);   
}

Control Defensor::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler ! 
    return ctrl; 
}

Defensor::~Defensor(){
    delete follower;
}

}
}
