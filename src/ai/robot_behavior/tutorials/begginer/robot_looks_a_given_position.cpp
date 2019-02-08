/*
    This file is part of SSL.
    
    Copyright 2019 Beltran Lila (lila.beltran@etu.u-bordeaux.fr)

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
#include "robot_looks_a_given_position.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {

Robot_looks_a_given_position::Robot_looks_a_given_position(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Robot_looks_a_given_position::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    annotations.clear();
    
    const rhoban_geometry::Point & current_position = linear_position();

    follower->set_following_position( current_position, this->angle ); 
    follower->update(time, robot, ball);
}

Control Robot_looks_a_given_position::control() const {
    Control ctrl = follower->control();
    return ctrl; 
}

Robot_looks_a_given_position::~Robot_looks_a_given_position(){
    delete follower;
}

RhobanSSLAnnotation::Annotations Robot_looks_a_given_position::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}

void Robot_looks_a_given_position::set_direction( const ContinuousAngle & angle ) {
	this->angle = angle;
}

void Robot_looks_a_given_position::set_direction( const Vector2d & vector ) {
	this->angle = detail::vec2angle( vector );
}

}
}
