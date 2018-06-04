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
    

    rhoban_geometry::Point oponent_goal_point = oponent_goal_center();
    Vector2d direction = oponent_goal_point - this->ball_position;
    direction = direction/direction.norm();
    double target_rotation = M_PI/2.0; // radian

    double target_radius_from_ball = 0.0;
    double error = 0.0;
    Vector2d target_position = this->ball_position - direction * (
         ai_data.constants.robot_radius + ai_data.constants.radius_ball + target_radius_from_ball + error
    );    

    if ( scalar_product(robot_position_point - ball_position, target_position - ball_position) > 0 ) {
        follower->avoid_the_ball(false);
    } else {
        follower->avoid_the_ball(true);
    }

    follower->set_following_position(target_position, target_rotation);
    

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
    ctrl.charge = true;
    ctrl.kick = true;
    // ctrl.spin = true; // We active the dribler ! 
    return ctrl; 
}

Striker::~Striker(){
    delete follower;
}

}
}
