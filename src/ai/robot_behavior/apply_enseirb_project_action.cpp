#include "apply_enseirb_project_action.h"
#include <math/tangents.h>
#include "factory.h"

namespace RhobanSSL {
namespace Robot_behavior {

Apply_enseirb_project_action::Apply_enseirb_project_action(
    Ai::AiData & ai_data,
    const enseirb::Action& action, double time, double dt
):
    ConsignFollower( ai_data ),
    follower( Factory::fixed_consign_follower(ai_data) ),
    action( action )
{ }

void Apply_enseirb_project_action::update(
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

    Vector2d linear_position(
        action.x, action.y
    );
    ContinuousAngle angular_position(action.t);
    follower->set_following_position(
        linear_position, angular_position
    );

    follower->update( time, robot, ball );   
}
Control Apply_enseirb_project_action::control() const {
    Control ctrl = follower->control(); 
    ctrl.kick = false;
    ctrl.active = true;
    ctrl.ignore = false;
    if(action.dribler){
        DEBUG("Dribbling not implemented.");
    }
    switch(action.kicker){
        case enseirb::Kicker::DO_NOTHING:
            break;
        case enseirb::Kicker::SHOOT:
            ctrl.kick = true;
            break;
        case enseirb::Kicker::LOBB:
            DEBUG("Lobbing not implemented.");
            break;
        default:
            assert(true);
    }
    return ctrl;
}


void Apply_enseirb_project_action::set_following_position(
    const Vector2d & position_to_follow,
    const ContinuousAngle & angle
){
    follower->set_following_position( position_to_follow, angle );
}

Apply_enseirb_project_action::~Apply_enseirb_project_action(){ }

}
}
