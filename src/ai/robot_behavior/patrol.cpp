/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

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
    it_s_time_to_change_the_zone(false),
    reverse_circuit(false)
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
             zone = (zone+ ( reverse_circuit ? -1:1) )%traject.size();
             it_s_time_to_change_the_zone = false;            
	    }
        //if( it_s_time_to_change_the_zone ){
        //    DEBUG("Erreur angulaire :" << target_rotation - angular_position());
        //    DEBUG("Erreur translation :" << target_position - linear_position());      
        //}
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


void Patrol::set_traject( const std::vector< rhoban_geometry::Point > & traject ){
    this->traject = std::vector< std::pair<rhoban_geometry::Point, ContinuousAngle> >(
        traject.size()
    );
    for( unsigned int i=0; i < traject.size(); i++ ){
        assert( norm( traject[(i+1)%traject.size()] - traject[i] ) != 0.0 );
        this->traject[i] = {
            traject[i],
            vector2angle( traject[(i+1)%traject.size()] - traject[i] )
        };
    }
}

Patrol* Patrol::two_way_trip_on_border( Ai::AiData& ai_data, bool left ){
    double sign = left ? -1.0:1.0;
    Patrol * res = new Patrol(ai_data);
    res->set_traject(
       {
           {
                rhoban_geometry::Point( -res->field_length()/4.0, sign*res->field_width()/4.0 ),
                ContinuousAngle(0.0)
            },
            {
                rhoban_geometry::Point( +res->field_length()/4.0, sign*res->field_width()/4.0 ),
                ContinuousAngle(0.0)
            }
       } 
    );
    res->set_waiting_time(0.7);
    res->see_the_ball(false);
    return res;
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
    res->set_waiting_time(1.0);
    res->see_the_ball(false);
    return res;
}

Patrol* Patrol::two_way_trip_on_width( Ai::AiData& ai_data, bool ally_side ){
    Patrol * res = new Patrol(ai_data);
    double sign = ally_side ? -1 : 1;
    //auto ally_center = res->center_ally_field();
    //auto opp_center = res->center_opponent_field();
    res->set_traject(
       {
            { 
                 rhoban_geometry::Point( sign*res->field_length()/4.0, -res->field_width()/4.0 ),
                 ContinuousAngle(M_PI/2.0)
            }, 
            {
                rhoban_geometry::Point( sign*res->field_length()/4.0, +res->field_width()/4.0 ),        
                ContinuousAngle(-M_PI/2.0)
            } 
       } 
    );
    res->set_waiting_time(1.0);
    res->see_the_ball(false);
    return res;
}


Patrol* Patrol::tour_of_the_field(
    Ai::AiData& ai_data, bool reverse_circuit 
){
    Patrol * res = new Patrol(ai_data);
    res->set_traject( res->center_quarter_field() );
    res->set_reverse(reverse_circuit);
    res->see_the_ball(false);
    res->set_waiting_time(1.0);
    return res;
}


    
void Patrol::set_reverse( bool reverse_circuit ){
    this->reverse_circuit = reverse_circuit;
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
