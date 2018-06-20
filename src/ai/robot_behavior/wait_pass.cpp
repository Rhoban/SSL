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
    distance_ball(12),
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
    const rhoban_geometry::Point & ball_position_now = ball.get_movement().linear_position( time );
    const rhoban_geometry::Point & ball_position_future = ball.get_movement().linear_position( time + 0.5 );
    annotations.addCross( ball_position_future.x, ball_position_future.y, "red");
    rhoban_geometry::Point target_position;
    rhoban_geometry::Point robot_position = robot.get_movement().linear_position( time );
    double target_rotation = detail::vec2angle(ball_position() - robot_position);
    distance_ball = (Vector2d(ball_position() - robot_position)).norm();

    if((Vector2d(ball_position_future - ball_position_now)).norm() > 0.3){


      Vector2d vect = ball_position_future - ball_position_now;
      double a = vect[1]/vect[0];
      double b = ball_position_future.getY() - a * ball_position_future.getX();
      // double eq_droite = robot_position.getY() - a*robot_position.getX() - b;

      if(vect[0] == 0 && vect[1] == 0){
        target_position = robot_position;
      }else{
        target_position.x = ( 1 / ( a + ( vect[0] / vect[1] ) ) ) * ( vect[0] / vect[1] * robot_position.x + robot_position.y - b );
        target_position.y = a*target_position.x + b;
      }
    }else{
      target_position = robot_position;
    }



    follower->avoid_the_ball(false);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control WaitPass::control() const {
    Control ctrl = follower->control();

    if (distance_ball < 0.8) {
      ctrl.spin = true; // We active the dribler !
    }else{
      ctrl.spin = false;
    }
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
