/*
    This file is part of SSL.
    
    Copyright 2018 Schmitz Etienne (hello@etienne-schmitz.com)

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
#include "begginer_go_corner.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {

Begginer_go_corner::Begginer_go_corner(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Begginer_go_corner::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    annotations.clear();
    
    // Set the robot_position to the right corner. (Use oponent_corner_left() for the left corner).
    const rhoban_geometry::Point & robot_position = oponent_corner_right();

    follower->set_following_position( robot_position, 0 ); 
    follower->update(time, robot, ball);
}

Control Begginer_go_corner::control() const {
    Control ctrl = follower->control();
    return ctrl; 
}

Begginer_go_corner::~Begginer_go_corner(){
    delete follower;
}

RhobanSSLAnnotation::Annotations Begginer_go_corner::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}

}
}
