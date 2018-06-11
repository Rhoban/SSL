#include "patrol.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


Patrol::Patrol(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) ),
    zone(0),
    _see_the_ball(false),
    waiting_time(0.0),
    last_time(ai_data.time),
    it_s_time_to_change_the_zone(false)
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
    
    ContinuousAngle target_rotation;
   
    if( traject.size() == 0 ){
        target_position = center_mark();
        target_rotation = ContinuousAngle(0.0);
    }else{
        target_position = traject.at( zone ).first;
        target_rotation = traject.at( zone ).second;

        if( not(it_s_time_to_change_the_zone) and  norm( robot_position - target_position ) < get_robot_radius() ){
             it_s_time_to_change_the_zone = true;            
             last_time = time;
        }
        if( it_s_time_to_change_the_zone and time - last_time > waiting_time  ){
             zone = (zone+1)%traject.size();
             it_s_time_to_change_the_zone = false;            
	    }
        if( it_s_time_to_change_the_zone ){
            DEBUG("Erreur angulaire :" << target_rotation - angular_position());
            DEBUG("Erreur translation :" << target_position - linear_position());      
        }
    }
    if( _see_the_ball ){
        Vector2d direction = ball_position() - robot_position;
        target_rotation = vector2angle( direction );
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

void Patrol::set_traject( const std::vector< std::pair<rhoban_geometry::Point, ContinuousAngle> > & traject ){
    this->traject = traject;
}

Patrol* Patrol::two_way_trip( Ai::AiData& ai_data ){
    Patrol * res = new Patrol(ai_data);
    auto ally_center = res->center_ally_field();
    auto opp_center = res->center_opponent_field();
    res->set_traject(
       {
           {ally_center, ContinuousAngle(0.0)},
           {opp_center, ContinuousAngle(0.0)}
       } 
    );
    res->see_the_ball(true);
    return res;
}

Patrol* Patrol::tour_of_the_field( Ai::AiData& ai_data ){
    Patrol * res = new Patrol(ai_data);
    //res->set_traject( res->center_quarter_field() );
    res->see_the_ball(true);
    return res;
}

Patrol* Patrol::test_translation_for_pid( Ai::AiData& ai_data ){
    Patrol * res = new Patrol(ai_data);
    res->set_traject( 
	{
            { 
                 rhoban_geometry::Point( -res->field_length()/4.0, -res->field_width()/4.0 ),
                 ContinuousAngle(M_PI/2.0)
            }, 
            {
                rhoban_geometry::Point( -res->field_length()/4.0, +res->field_width()/4.0 ),        
                ContinuousAngle(M_PI/2.0)
            }, 
        }
    );
    res->set_waiting_time(5.0);
    res->see_the_ball(false);
    return res; 
}	

Patrol* Patrol::test_rotation_for_pid( Ai::AiData& ai_data ){
    Patrol * res = new Patrol(ai_data);
    res->set_traject( 
	{
            { 
                 rhoban_geometry::Point( -res->field_length()/4.0, -res->field_width()/4.0 ),
                 ContinuousAngle(0.0)
            }, 
            {
                rhoban_geometry::Point( -res->field_length()/4.0, -res->field_width()/4.0 ),        
                ContinuousAngle(M_PI)
            }, 
        }
    );
    res->set_waiting_time(5.0);
    res->see_the_ball(false);
    return res; 
}	




RhobanSSLAnnotation::Annotations Patrol::get_annotations() const {
    return follower->get_annotations();
}

void Patrol::see_the_ball(bool value){
    _see_the_ball = value;
}

void Patrol::set_waiting_time( double time ){
    waiting_time = time;
}

}
}
