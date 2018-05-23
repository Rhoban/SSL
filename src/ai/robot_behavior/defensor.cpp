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
    Vector2d defender_pos = this->ball_position;
    
    follower->set_following_position(defender_pos, goal_rotation );

    follower->update(time, robot, ball);   
}


Control Defensor::control() const {
    return follower->control();
}

Defensor::~Defensor(){
    delete follower;
}

}
}
