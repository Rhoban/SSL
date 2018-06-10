#include "patrol.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


Patrol::Patrol(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) ),
    zone(0)
{
}

void Patrol::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    rhoban_geometry::Point target_position;

    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );
    
    Vector2d direction = ball_position() - robot_position;
    ContinuousAngle target_rotation = vector2angle( direction );
   
    if( traject.size() == 0 ){
        target_position = center_mark();
    }else{
        target_position = traject.at( zone );

        if( norm( robot_position - target_position ) < get_robot_radius() ){
            zone = (zone+1)%traject.size();
        }
    }
    
    follower->avoid_the_ball(true);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control Patrol::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler !
    ctrl.kick = false; 
    return ctrl; 
}

Patrol::~Patrol(){
    delete follower;
}

void Patrol::set_traject( const std::vector< rhoban_geometry::Point > & traject ){
    this->traject = traject;
}

Patrol* Patrol::two_way_trip( Ai::AiData& ai_data ){
    Patrol * res = new Patrol(ai_data);
    auto ally_center = res->center_ally_field();
    auto opp_center = res->center_opponent_field();
    res->set_traject( { ally_center, opp_center } );
    return res;
}

Patrol* Patrol::tour_of_the_field( Ai::AiData& ai_data ){
    Patrol * res = new Patrol(ai_data);
    res->set_traject( res->center_quarter_field() );
    return res;
}


}
}
