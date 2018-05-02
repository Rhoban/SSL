#include "apply_enseirb_project_action.h"
#include <math/tangents.h>

namespace RhobanSSL {
namespace Robot_behavior {

Apply_enseirb_project_action::Apply_enseirb_project_action( const enseirb::Action& action, double time, double dt ):
    PositionFollower(time, dt),
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
    this->set_following_position(
        linear_position, angular_position
    );

    PositionFollower::update_control( time );   
}
Control Apply_enseirb_project_action::control() const {
    Control ctrl = PositionFollower::control(); 
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

}
}
