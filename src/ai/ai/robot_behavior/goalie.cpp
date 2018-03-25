#include "goalie.h"
#include <math/tangents.h>
#include <math/eigen_convertion.h>

namespace RhobanSSL {

Eigen::Vector2d Goalie::calculate_goal_position(
    const Eigen::Vector2d & ball_position,
    const Eigen::Vector2d & poteau_droit,
    const Eigen::Vector2d & poteau_gauche,
    double goalie_radius
){
    rhoban_geometry::Point defender_position = rhoban_geometry::center_of_cone_incircle(
	eigen2point(ball_position),
	eigen2point(poteau_droit), 
	eigen2point(poteau_gauche), 
	goalie_radius
    );
    return point2eigen( defender_position );
}


Goalie::Goalie(
    const Eigen::Vector2d & left_post_position,
    const Eigen::Vector2d & right_post_position,
    const Eigen::Vector2d & waiting_goal_position,
    double penalty_rayon,
    double goalie_radius,
    double time, double dt
):
    PositionFollower(time, dt)
{
    this->left_post_position = left_post_position;
    this->right_post_position = right_post_position;
    this->waiting_goal_position = waiting_goal_position;
    this->goal_center = (left_post_position + right_post_position)/2.0;
    this->penalty_rayon = penalty_rayon;
    this->goalie_radius = goalie_radius;
}

void Goalie::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Now 
    //  this->robot_linear_position
    //  this->ball_position
    //  this->robot_angular_position 
    // are all avalaible
    

    double goal_rotation = detail::vec2angle(ball_position - robot_linear_position);

    Eigen::Vector2d defender_pos = calculate_goal_position(
        ball_position, right_post_position, left_post_position,
        goalie_radius
    );

    if( (defender_pos - goal_center).norm() > penalty_rayon ){
        defender_pos = waiting_goal_position;
    }

    this->set_following_position(defender_pos, goal_rotation );

    PositionFollower::update_control(time);   
}

}
