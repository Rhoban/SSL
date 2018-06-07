#include "passive_defensor.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>

namespace RhobanSSL {
namespace Robot_behavior {


Passive_defensor::Passive_defensor(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) ),
    barycenter(.5)
{
}

void Passive_defensor::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    const Ai::Robot& ennemy = get_robot( robot_to_obstale_id, robot_to_obstale_team );
    rhoban_geometry::Point ennemy_position = ennemy.get_movement().linear_position(time);
 
    rhoban_geometry::Point target_position = vector2point(
       Vector2d(  ball_position() ) * barycenter
        +
        Vector2d( ennemy_position ) * (1.0-barycenter)
    );

    ContinuousAngle target_rotation = detail::vec2angle( Vector2d( ball_position() - target_position ) );
 
    follower->avoid_the_ball(false); 
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);   
}

Control Passive_defensor::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler ! 
    return ctrl; 
}

Passive_defensor::~Passive_defensor(){
    delete follower;
}

void Passive_defensor::set_robot_to_obstacle( int robot_id, Vision::Team team ){
    this->robot_to_obstale_id = robot_id;
    this->robot_to_obstale_team = team;
}

void Passive_defensor::set_barycenter( double barycenter ){
    assert( 0<= barycenter and barycenter <= 1.0 );
    this->barycenter = barycenter;
}

}
}
