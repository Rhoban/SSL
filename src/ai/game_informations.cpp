/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
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

#include "game_informations.h"

namespace RhobanSSL {

GameInformations::GameInformations( Ai::AiData & ai_data ):
    ai_data( ai_data)
{ }

GameInformations::~GameInformations(){ }

double GameInformations::time() const {
    return ai_data.time;
}


rhoban_geometry::Point GameInformations::ally_goal_center() const {
    return  rhoban_geometry::Point( - ai_data.field.fieldLength/2.0, 0.0 );

}

rhoban_geometry::Point GameInformations::oponent_goal_center() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, 0.0 );
}

rhoban_geometry::Point GameInformations::center_mark() const {
    return rhoban_geometry::Point( 0.0, 0.0 );
}

rhoban_geometry::Point GameInformations::oponent_corner_right() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, - ai_data.field.fieldWidth/2.0 );
}

rhoban_geometry::Point GameInformations::oponent_corner_left() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, ai_data.field.fieldWidth/2.0 );
}

const Ai::Robot & GameInformations::get_robot( int robot_id, Vision::Team team ) const {
    return ai_data.robots.at(team).at(robot_id);
}

std::vector<int> GameInformations::get_robot_in_line( const rhoban_geometry::Point p1, const rhoban_geometry::Point p2, Vision::Team team, double seuil ) const{
  std::vector<int> v;
  Vector2d vect = p1 - p2;
  for (size_t i = 0; i <= 7; i++) {
    const Ai::Robot & robot = get_robot( i,  team );
    if(robot.is_present_in_vision()){
      const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( time() );

      double a = vect[1]/vect[0];
      double b = p1.getY() - a * p1.getX();
      double eq_droite = robot_position.getY() - a*robot_position.getX() - b;

      double robot_line = (Vector2d(robot_position - p2)).norm();
      double p1_line = (Vector2d(p1 - p2)).norm();
      double diff = robot_line - p1_line;

      if (fabs(eq_droite) <= seuil && diff < 0 ) {
        v.push_back(i);
      }
    }
  }
  return v;
}


const Ai::Ball & GameInformations::ball() const {
  return ai_data.ball;
}

rhoban_geometry::Point GameInformations::ball_position() const{
  return ball().get_movement().linear_position(time());
}


rhoban_geometry::Point GameInformations::center_ally_field() const {
    return rhoban_geometry::Point( -ai_data.field.fieldLength/4.0, 0.0 );
}
rhoban_geometry::Point GameInformations::center_opponent_field() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/4.0, 0.0 );
}

double GameInformations::get_robot_radius() const {
    return ai_data.constants.robot_radius;
}

double GameInformations::get_ball_radius() const {
    return ai_data.constants.radius_ball;
}

std::vector<rhoban_geometry::Point> GameInformations::center_quarter_field() const {
    return std::vector<rhoban_geometry::Point>(
        {
            rhoban_geometry::Point( ai_data.field.fieldLength/4.0, ai_data.field.fieldWidth/4.0 ),
            rhoban_geometry::Point( ai_data.field.fieldLength/4.0, -ai_data.field.fieldWidth/4.0 ),
            rhoban_geometry::Point( -ai_data.field.fieldLength/4.0, -ai_data.field.fieldWidth/4.0 ),
            rhoban_geometry::Point( -ai_data.field.fieldLength/4.0, ai_data.field.fieldWidth/4.0 ),
        }
    );
}

double GameInformations::field_width() const{
 return ai_data.field.fieldWidth;
}

double GameInformations::field_length() const{
 return ai_data.field.fieldLength;
}



rhoban_geometry::Point GameInformations::field_SW() const{
    return rhoban_geometry::Point( -ai_data.field.fieldLength/2.0, -ai_data.field.fieldWidth/2.0  );
}
rhoban_geometry::Point GameInformations::field_NW() const{
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, -ai_data.field.fieldWidth/2.0  );
}
rhoban_geometry::Point GameInformations::field_NE() const{
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, ai_data.field.fieldWidth/2.0  );
}
rhoban_geometry::Point GameInformations::field_SE() const{
    return rhoban_geometry::Point( -ai_data.field.fieldLength/2.0, ai_data.field.fieldWidth/2.0  );
}

Box GameInformations::field() const {
    return Box ( field_SW(), field_NE() );
}

Box GameInformations::ally_penalty_area() const {
    return Box(
        {
            - ai_data.field.fieldLength/2.0,
            - ai_data.field.penaltyAreaWidth/2.0
        },
        {
            - ( ai_data.field.fieldLength/2.0 - ai_data.field.penaltyAreaDepth ),
            ai_data.field.penaltyAreaWidth/2.0
        }
    );
}

Box GameInformations::opponent_penalty_area() const {
    return Box(
        {
            ( ai_data.field.fieldLength/2.0 - ai_data.field.penaltyAreaDepth ),
            - ai_data.field.penaltyAreaWidth/2.0
        },
        {
            ai_data.field.fieldLength/2.0,
            ai_data.field.penaltyAreaWidth/2.0
        }
    );
}

double GameInformations::penalty_area_width() const{
    return ai_data.field.penaltyAreaWidth;
}

double GameInformations::penalty_area_depth() const {
    return ai_data.field.penaltyAreaDepth;
}

};
