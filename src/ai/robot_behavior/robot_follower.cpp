#include "robot_follower.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <math/ContinuousAngle.h>

namespace RhobanSSL {
namespace Robot_behavior {


RobotFollower::RobotFollower(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    robot_to_follow_id(-1),
    team(Vision::Team::Ally),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void RobotFollower::update(
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
    
        
    const Ai::Robot & robot_to_follow = get_robot( robot_to_follow_id, robot_to_follow_team );
    rhoban_geometry::Point position = robot_to_follow.get_movement().linear_position( time );
    rhoban_geometry::Point target_position = position + translation; 
    ContinuousAngle target_rotation(0.0);

    follower->avoid_the_ball(true);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);   
}

Control RobotFollower::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler ! 
    return ctrl; 
}

RobotFollower::~RobotFollower(){
    delete follower;
}


void RobotFollower::declare_robot_to_follow( int robot_id, const Vector2d & translation, Vision::Team team ){
    robot_to_follow_id = robot_id;
    this->translation = translation;
    robot_to_follow_team = team;
}


}
}
