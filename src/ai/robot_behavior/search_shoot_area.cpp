/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#include "search_shoot_area.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>
#include <core/print_collection.h>

namespace RhobanSSL {
namespace Robot_behavior {


SearchShootArea::SearchShootArea(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    period(3),
    last_time_changement(0),
    obstructed_view(-1),
    follower( Factory::fixed_consign_follower(ai_data) )
{
  p1 = vector2point(
      Vector2d(
        oponent_corner_left() - rhoban_geometry::Point(1, 1)
      )
  );
  p2 = vector2point(
      Vector2d(
        center_mark() - rhoban_geometry::Point(-1, 2)
      )
  );
}

void SearchShootArea::update(
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

    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( time );

    Vector2d opponent_goal_robot_vector = robot_position - oponent_goal_center();
    annotations.addArrow( robot_position, oponent_goal_center(), "red" );

    double seuil = 0.4;
    obstructed_view = 0;
    const std::vector<int> v_obstruct = get_robot_in_line( robot_position, oponent_goal_center(), Vision::Team::Opponent, seuil);
    if(!v_obstruct.empty()){
      obstructed_view = 1;
    }
    // double seuil = 0.4;
    // obstructed_view = 0;
    // for (size_t i = 0; i <= 7; i++) {
    //   const Ai::Robot & robot_opponent = get_robot( i,  Vision::Team::Opponent );
    //   if(robot_opponent.is_present_in_vision()){
    //     const rhoban_geometry::Point & robot_position_opponent = robot_opponent.get_movement().linear_position( time );
    //
    //     double a = opponent_goal_robot_vector[1]/opponent_goal_robot_vector[0];
    //     double b = robot_position.getY() - a * robot_position.getX();
    //     double eq_droite = robot_position_opponent.getY() - a*robot_position_opponent.getX() - b;
    //
    //     double robot_opponent_goal = (Vector2d(robot_position_opponent - oponent_goal_center())).norm();
    //     double robot_goal = (Vector2d(robot_position - oponent_goal_center())).norm();
    //     double diff = robot_opponent_goal - robot_goal;
    //
    //
    //     if (fabs(eq_droite) <= seuil && diff < 0 ) {
    //       obstructed_view += 1;
    //     }
    //   }
    // }

    double pos_x = robot_position.getX();
    double pos_y = robot_position.getY();
    Vector2d ball_robot_vector = ball_position() - robot_position;
    ContinuousAngle target_rotation = vector2angle( ball_robot_vector  );

    if (obstructed_view == 0 && pos_x <= std::max(p1.x, p2.x) && pos_x > std::min(p1.x, p2.x) && pos_y <= std::max(p1.y, p2.y) && pos_y > std::min(p1.y, p2.y))  {
      // DEBUG( "robot_position : " << robot_position );
      target_position = robot_position;
      // DEBUG( "target_position : " << target_position );
    }
    else{
      if( time > last_time_changement + period ){
        std::uniform_real_distribution<double> distribution_x(p1.x, p2.x);
        std::uniform_real_distribution<double> distribution_y(p1.y, p2.y);
        target_position = rhoban_geometry::Point(
          distribution_x(generator), distribution_y(generator)
        );
        last_time_changement = time;
      }
    }

    // target_position = robot_position;
    // target_position = robot_position;

    annotations.addCross( target_position.x, target_position.y );

    follower->avoid_the_ball(true);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control SearchShootArea::control() const {
    Control ctrl = follower->control();
    return ctrl;
}

void SearchShootArea::declare_area( rhoban_geometry::Point p1, rhoban_geometry::Point p2){
  this->p1 = p1;
  this->p2 = p2;
}

SearchShootArea::~SearchShootArea(){
    delete follower;
}


RhobanSSLAnnotation::Annotations SearchShootArea::get_annotations() const {
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations( this->annotations );
  annotations.addAnnotations( follower->get_annotations() );
  return annotations;
}

}
}
