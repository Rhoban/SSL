/*
    This file is part of SSL.

    Copyright 2019 SCHMITZ Etienne (hello@etienne-schmitz.com)

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

#include "see_ball.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {
namespace Beginner
{
See_ball::See_ball(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void See_ball::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    annotations.clear();
    
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );

    // REVIEW XM : rename direction variable by robot_to_ball or robot_ball ?
    Vector2d direction = ball_position() - robot_position;
    // REVIEW XM : rename target_rotation by target_angular_position ?
    ContinuousAngle target_rotation = vector2angle( direction );
    
    follower->set_following_position(robot_position, target_rotation );
    follower->update(time, robot, ball);
}

Control See_ball::control() const {
    Control ctrl = follower->control();
    return ctrl; 
}

See_ball::~See_ball(){
    delete follower;
}

RhobanSSLAnnotation::Annotations See_ball::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}


}
}
}
