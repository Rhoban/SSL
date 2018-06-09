#include "concept_proof_spinner.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>

namespace RhobanSSL {
namespace Robot_behavior {


Concept_proof_spinner::Concept_proof_spinner(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) ),
    save_ball_position(false),
    go_to_home(false)
{
}

void Concept_proof_spinner::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );

    rhoban_geometry::Point pos = robot.get_movement().linear_position(time);
    Vector2d direction = Vector2d(ball_position()) - Vector2d(pos);
    direction = direction / direction.norm();
    
    rhoban_geometry::Point target_position;

    if(
        not(save_ball_position) && (
            norm( Vector2d( ball_position()) - Vector2d(pos ) ) < 5*ai_data.constants.radius_ball  
        )
    ){
        save_ball_position = true;
        ball_pos = ball_position();
    }
    if( save_ball_position && norm( Vector2d( ball_pos )  - Vector2d( pos ) ) < ai_data.constants.radius_ball/2.0 ){
        go_to_home = true;   
    }

    if( not(go_to_home) ){
        target_position = (
            ball_position() + 
            direction * ai_data.constants.robot_radius
        );
    }else{
        target_position = ally_goal_center();
    }
    ContinuousAngle angle = vector2angle( direction  );

    follower->avoid_the_ball(false); 
    follower->set_following_position(target_position, angle);
    follower->update(time, robot, ball);   
}

Control Concept_proof_spinner::control() const {
    Control ctrl = follower->control();
    ctrl.spin = true;
    return ctrl; 
}

Concept_proof_spinner::~Concept_proof_spinner(){
    delete follower;
}

}
}
