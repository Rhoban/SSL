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

#ifndef __GAME_INFORMATIONS__H__
#define __GAME_INFORMATIONS__H__

#include <AiData.h>
#include <math/box.h>

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
    std::vector<int> get_robot_in_line( const rhoban_geometry::Point p1, const rhoban_geometry::Point p2, Vision::Team team = Vision::Team::Opponent, double seuil = 0.4 ) const ;
    std::vector<int> get_robot_in_line( const rhoban_geometry::Point p1, const rhoban_geometry::Point p2, double seuil ) const ;
    std::pair<rhoban_geometry::Point, double> find_goal_best_move( const rhoban_geometry::Point point, const rhoban_geometry::Point goal = rhoban_geometry::Point(66,66) ) const ;

    bool infra_red(  int robot_id, Vision::Team team = Vision::Team::Ally ) const;
    int get_nearest_ball() const ;
    int get_nearest_ball( Vision::Team team ) const ;
    int get_nearest_point( Vision::Team team, rhoban_geometry::Point point ) const ;

    double threat_robot( int id_robot, Vision::Team team = Vision::Team::Opponent) const ;
    std::vector<double> threat(  Vision::Team team = Vision::Team::Opponent ) const ;
    int id_threat_max( Vision::Team team ) const;
    int id_threat_max_2( Vision::Team team ) const; // second threat max
    int id_threat_max( ) const;


    const Ai::Ball & ball() const ;
    rhoban_geometry::Point ball_position() const ;

    std::vector<rhoban_geometry::Point> center_quarter_field() const ;

    rhoban_geometry::Point center_ally_field() const ;
    rhoban_geometry::Point center_opponent_field() const ;
    double get_robot_radius() const;
    double get_ball_radius() const;
    double field_width() const;
    double field_length() const;
    double penalty_area_width() const;
    double penalty_area_depth() const;

    rhoban_geometry::Point field_SW() const;
    rhoban_geometry::Point field_NW() const;
    rhoban_geometry::Point field_NE() const;
    rhoban_geometry::Point field_SE() const;

    Box field() const;
    Box ally_penalty_area() const;
    Box opponent_penalty_area() const;

};


}

#endif
