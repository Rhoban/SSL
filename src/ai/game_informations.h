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

#ifndef __GAME_INFORMATIONS__H__
#define __GAME_INFORMATIONS__H__

#include <AiData.h>

namespace RhobanSSL {

class GameInformations {
    private:
    Ai::AiData & ai_data;

    public:
    GameInformations( Ai::AiData & ai_data );
    virtual ~GameInformations();

    double time() const;
    rhoban_geometry::Point ally_goal_center() const ;
    rhoban_geometry::Point oponent_goal_center() const ;
    rhoban_geometry::Point center_mark() const ;
    rhoban_geometry::Point oponent_corner_right() const ;
    rhoban_geometry::Point oponent_corner_left() const ;
    const Ai::Robot & get_robot( int robot_id, Vision::Team team = Vision::Team::Ally ) const ;

    const Ai::Ball & ball() const ;
    rhoban_geometry::Point ball_position() const ;

    std::vector<rhoban_geometry::Point> center_quarter_field() const ;

    rhoban_geometry::Point center_ally_field() const ;
    rhoban_geometry::Point center_opponent_field() const ;
    double get_robot_radius() const;
    double get_ball_radius() const;

};


}

#endif
