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
    
    rhoban_geometry::Point ally_goal_point = ally_goal_center();
    rhoban_geometry::Point left_post_position = rhoban_geometry::Point( -ai_data.field.fieldLength / 2.0, ai_data.field.goalWidth / 2.0 );
    rhoban_geometry::Point right_post_position = rhoban_geometry::Point( -ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0 );

    Vector2d ball_goal_vector = ally_goal_point - ball_position();
    Vector2d ball_robot_vector = robot_position - ball_position();
    Vector2d ball_l_post_vector = left_post_position - ball_position();
    Vector2d ball_r_post_vector = right_post_position - ball_position();

    ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
    ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();
    ball_l_post_vector = ball_l_post_vector / ball_l_post_vector.norm();
    ball_r_post_vector = ball_r_post_vector / ball_r_post_vector.norm();


    double scalar_ball_robot = scalar_product( ball_robot_vector , ball_goal_vector );

    //target_radius_from_ball = (ai_data.constants.robot_radius / 2) / std::tan(goal_visible_angle / 2);

    if ( scalar_ball_robot < 0 ) {
        follower->avoid_the_ball(true);
    } else {
        follower->avoid_the_ball(false);
        //target_radius_from_ball = (ai_data.constants.robot_radius) / std::tan(std::acos(goal_visible_angle) / 2);
    }


    rhoban_geometry::Point target_position = rhoban_geometry::center_of_cone_incircle(ball_position(), left_post_position, right_post_position, ai_data.constants.robot_radius);

    //Vector2d target_position = Vector2d(ball_position()) + ball_goal_vector * (target_radius_from_ball);

    double target_rotation = detail::vec2angle(-ball_goal_vector);
    

    Vector2d target_goal_vector = ally_goal_point - target_position;
    double limit_defense_area_radius = 1.4;

    if ( target_goal_vector.norm() < limit_defense_area_radius or scalar_product( target_goal_vector, ball_goal_vector ) < 0 ) {
        target_position = ally_goal_point + ball_goal_vector * (-limit_defense_area_radius);
    }

    follower->set_following_position(Vector2d(target_position), target_rotation);
    follower->update(time, robot, ball);
}

Control Defensor::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler !
    ctrl.kick = false; 
    return ctrl; 
}

Defensor::~Defensor(){
    delete follower;
}

}
}
