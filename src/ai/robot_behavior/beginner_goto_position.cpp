/*
    This file is part of SSL.

    Copyright 2019 Beltran Lila (lila.bebltran@etu.u-bordeaux.fr)

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

#include "beginner_goto_position.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {

Beginner_goto_position::Beginner_goto_position(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Beginner_goto_position::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    annotations.clear();
    
    rhoban_geometry::Point robot_position = rhoban_geometry::Point(0.0, 0.0);

    ContinuousAngle angle = 0.0;

    follower->set_following_position( robot_position,angle );
    follower->update(time, robot, ball);
}

Control Beginner_goto_position::control() const {
    Control ctrl = follower->control();
    return ctrl; 
}

Beginner_goto_position::~Beginner_goto_position(){
    delete follower;
}

RhobanSSLAnnotation::Annotations Beginner_goto_position::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}



}
} 
