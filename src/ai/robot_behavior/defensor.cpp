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
    //  this->ball_position
    //  this->robot_angular_position 
    // are all avalaible
    

    double goal_rotation = M_PI/2.0; // radian
    Vector2d our_goal_center( - ai_data.field.fieldLength/2.0, 0.0  );
    Vector2d direction = our_goal_center - this->ball_position;
    direction = direction/direction.norm();

    double error = 0.03;
    Vector2d defender_pos =  this->ball_position + direction * (
         ai_data.constants.robot_radius + ai_data.constants.radius_ball + error
    );
    
    follower->set_following_position(defender_pos, goal_rotation );

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
