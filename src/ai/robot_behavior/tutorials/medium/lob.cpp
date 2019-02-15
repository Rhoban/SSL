/*
    This file is part of SSL.
    
    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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
#include "lob.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {
namespace Medium {
Lob::Lob(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Lob::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    annotations.clear();

    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );
    Vector2d ball_robot_vector = ball_position() - robot_position;
    rhoban_geometry::Point target_position = ball_position();

    follower->avoid_the_ball(false);
    follower->set_following_position(target_position, vector2angle(ball_robot_vector)); 
    follower->update(time, robot, ball);
}

Control Lob::control() const {
    const rhoban_geometry::Point & robot_position = linear_position();
    Vector2d ball_robot_vector = robot_position - ball_position();
    double dist = ball_robot_vector.norm();
    Control ctrl = follower->control();

    if(dist < 0.1) {
        ctrl.chipKick = true;
    }
    return ctrl; 
}

Lob::~Lob(){
    delete follower;
}

RhobanSSLAnnotation::Annotations Lob::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}

}
}
}