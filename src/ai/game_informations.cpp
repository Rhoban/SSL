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








};
