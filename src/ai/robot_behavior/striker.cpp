#include "striker.h"
#include <math/tangents.h>
#include <math/vector2d.h>

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
    //  this->ball_position
    //  this->robot_angular_position 
    // are all avalaible
    
    //int robot_id = 2;
    //const Robots_table & robot_table = ai_data.robots.at(Vision::Team::Ally);
    //const Ai::Robot & robot = robot_table.at(robot_id);
    
    const rhoban_geometry::Point & robot_position_point = robot.get_movement().linear_position( ai_data.time );
    

    double goal_rotation = M_PI/2.0; // radian
    Vector2d our_goal_center( - ai_data.field.fieldLength/2.0, 0.0 );
    Vector2d direction = our_goal_center - this->ball_position;
    direction = direction/direction.norm();

    double target_radius_from_ball = 0.5;
    double error = 0.03;
    Vector2d striker_pos =  this->ball_position + direction * (
         ai_data.constants.robot_radius + ai_data.constants.radius_ball + target_radius_from_ball + error
    );    

    if ( robot_position_point.getX() < this->ball_position.getX() ) {
        follower->avoid_the_ball(false);
    } else {
        follower->avoid_the_ball(true);    
    }

    follower->set_following_position(striker_pos, goal_rotation);
    

    #if 0    

    else {
        if (robot_position_point.getY() < this->ball_position.getY()) {
        follower->set_following_position(defender_pos, goal_rotation );
        }
        else {
        follower->set_following_position(defender_pos, goal_rotation );
        }
    }
#endif
    follower->update(time, robot, ball);   
}


Control Striker::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler ! 
    return ctrl; 
}

Striker::~Striker(){
    delete follower;
}

}
}
