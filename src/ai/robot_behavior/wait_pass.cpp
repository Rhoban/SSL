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

#include "wait_pass.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <math/ContinuousAngle.h>

namespace RhobanSSL {
namespace Robot_behavior {


WaitPass::WaitPass(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void WaitPass::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Now
    //  this->robot_linear_position
    //  this->robot_angular_position
    // are all avalaible

    annotations.clear();
    // rhoban_geometry::Point target_position = robot.get_movement().linear_position( time );
    const rhoban_geometry::Point & target_position = ball.get_movement().linear_position( time + 1 );
    annotations.addCross( target_position.x, target_position.y, "red");
    rhoban_geometry::Point robot_position = robot.get_movement().linear_position( time );
    double target_rotation = detail::vec2angle(ball_position() - robot_position);

    follower->avoid_the_ball(true);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control WaitPass::control() const {
    Control ctrl = follower->control();
    ctrl.spin = true; // We active the dribler !
    return ctrl;
}

WaitPass::~WaitPass(){
    delete follower;
}

RhobanSSLAnnotation::Annotations WaitPass::get_annotations() const {
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations( this->annotations );
  annotations.addAnnotations( follower->get_annotations() );
  return annotations;;
}

}
}
