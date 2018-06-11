#include "example.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


Example::Example(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Example::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    annotations.clear();
    
    const rhoban_geometry::Point & target_position = center_mark();
    
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );

    // On ajoute un text à la position du robot.
    //annotations.addText("Exemple :)", robot_position, "blue");

    Vector2d direction = ball_position() - robot_position;
    ContinuousAngle target_rotation = vector2angle( direction );
    
    follower->avoid_the_ball(true);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control Example::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler !
    ctrl.kick = false; 
    return ctrl; 
}

Example::~Example(){
    delete follower;
}

RhobanSSLAnnotation::Annotations Example::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}



}
}
