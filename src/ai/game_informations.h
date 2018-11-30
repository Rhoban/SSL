/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)
    Copyright 2018 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

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

    /**
     * @brief return ally goal center
     * @return const rhoban_geometry::Point
     */
    rhoban_geometry::Point ally_goal_center() const ;
    /**
     * @brief oponent_goal_center
     * @return const rhoban_geometry::Point
     */
    rhoban_geometry::Point oponent_goal_center() const ;
    /**
     * @brief center_mark
     * @return const rhoban_geometry::Point
     */
    rhoban_geometry::Point center_mark() const ;
    /**
     * @brief return opponant right corner
     * @return const rhoban_geometry::Point
     */
    rhoban_geometry::Point oponent_corner_right() const ;
    /**
     * @brief return opponant left corner
     * @return const rhoban_geometry::Point
     */
    rhoban_geometry::Point oponent_corner_left() const ;
    /**
     * @brief return the robot's reference that correspond to the given
     * robot_id and team in parameters
     * @param robot_id
     * @param team
     * @return robot's reference
     */
    const Ai::Robot & get_robot( int robot_id, Vision::Team team = Vision::Team::Ally ) const ;
    /**
     * @brief store all robots in the team 'team' which are at a distance
     * between the line describe by 'p1' and 'p2' less than the 'distance'
     * parameter
     * @param p1
     * @param p2
     * @param team ( Vision::Team::Opponent ou Vision::Team::Ally)
     * @param distance (usually the robot radius)
     * @param result
     */
    void get_robot_in_line(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
        Vision::Team team, double distance,
        std::vector<int> & result
    ) const ;
    /**
     * @brief same as void get_robot_in_line() but return the result
     * instead of storing it in a variable
     * @param p1
     * @param p2
     * @param team  ( Vision::Team::Opponent ou Vision::Team::Ally)
     * @param distance (usually the robot radius)
     * @return vector<int>
     */
    std::vector<int> get_robot_in_line(
        const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
        Vision::Team team = Vision::Team::Opponent, double distance = 0.4
    ) const ;
    /**
     * @brief same as vector<int> get_robot_in_line() but doesn't
     * consider the robot's team
     * @param p1
     * @param p2
     * @param distance (usually the robot radius)
     * @return vector<int>
     */
    std::vector<int> get_robot_in_line(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
        double distance
    ) const ;
    /**
     * @brief find the "ideal" position that maximazes the chance to score
     * in the targeted goal in parameter
     * @note Attack algorithm
     * @param point  (usually the ball position)
     * @param goal
     * @return pair<rhoban_geometry::Point, double>
     * (double correspond of his "efficiency rate" )
     */
    std::pair<rhoban_geometry::Point, double> find_goal_best_move( const rhoban_geometry::Point point, const rhoban_geometry::Point goal = rhoban_geometry::Point(66,66) ) const ;
    /**
     * @brief return true if the ball is closed to the kicker.
     * In front of the kicker there is an infrared captor,
     * when the ball is on the captor, the captor return true.
     * In fact we don't use it to kick. Indeed we can ask the robot
     * to kick when infrared is not on, in that case the robot will
     * kick as soon as the infrared will be on
     * @param robot_id
     * @param team
     * @return true/false
     */
    bool infra_red(  int robot_id, Vision::Team team = Vision::Team::Ally ) const;
    /**
     * @brief find the robot id of the closest robot to the ball
     * @return robot index ( -1 if not found)
     */
    int get_nearest_ball() const ;
    /**
     * @brief find the robot id of the closest robot to the ball
     * from the team 'team'
     * @param team ( Vision::Team::Opponent or Vision::Team::Ally)
     * @return robot index (-1 if not found)
     */
    int get_nearest_ball( Vision::Team team ) const ;
    /**
     * @brief find the robot id of the closest robot to the 'point'
     * from the team 'team'
     * @param team ( Vision::Team::Opponent or Vision::Team::Ally)
     * @param point
     * @return robot index (-1 if not found)
     */
    int get_nearest_point( Vision::Team team, rhoban_geometry::Point point ) const ;
    /**
     * @brief find the distance between a robot and the ally goal center
     * @note Defense algorithm
     * @param id_robot
     * @param team (opponent by default)
     * @return distance
     */
    double get_robot_distance_from_ally_goal_center( int id_robot, Vision::Team team = Vision::Team::Opponent) const ;
    /**
     * @brief find the distance between all robots in the team 'team' and
     * the ally goal center
     * @note Defense algorithm
     * @param team (opponent by default
     * @return vector [0] = distance first robot, [1] = distance second robot
     * ...
     */
    std::vector<double> threat(  Vision::Team team = Vision::Team::Opponent ) const ;
    /**
     * @brief find the biggest threat belonging to the team 'team'
     * which correspond to the closest opposant's robot to the ally goal
     * center
     * @note Defense algorithm
     * @param team
     * @return robot id
     */
    int id_threat_max( Vision::Team team ) const;
    /**
     * @brief find the second biggest threat belonging to the team 'team'
     * which correspond to the second closest opposant's robot to the ally goal
     * center
     * @note Defense algorithm
     * @param team
     * @return robot id
     */
    int id_threat_max_2( Vision::Team team ) const; // second threat max
    /**
     * @brief  find the biggest threat in the field
     * doesn't consider any team side
     * @note Defense algorithm
     * @return robot index
     */
    int id_threat_max( ) const;

    /**
     * @brief return ball reference
     * @return const Ai::Ball&
     */
    const Ai::Ball & ball() const ;
    /**
     * @brief return the current ball_position
     * @return const rhoban_geometry::Point
     */
    rhoban_geometry::Point ball_position() const ;
    /**
     * @brief return all the quarter's of the field
     * @return const vector<rhoban_geometry::Point>
     */
    std::vector<rhoban_geometry::Point> center_quarter_field() const ;
    /**
     * @brief return center ally field
     * @return const rhoban_geometry::Point
     */
    rhoban_geometry::Point center_ally_field() const ;
    /**
     * @brief return center opponant field
     * @return const rhoban_geometry::Point
     */
    rhoban_geometry::Point center_opponent_field() const ;
    /**
     * @brief return robot radius
     * @return const double
     */
    double get_robot_radius() const;    
    /**
     * @brief return ball radius constant
     * @return cosnt double
     */
    double get_ball_radius() const;    
    /**
     * @brief return field width
     * @return double
     */
    double field_width() const;
    /**
     * @brief return field length
     * @return double
     */
    double field_length() const;
    /**
     * @brief return penalty area width
     * @return double
     */
    double penalty_area_width() const;
    /**
     * @brief return penalty area depth
     * @return double
     */
    double penalty_area_depth() const;
    /**
     * @brief return the South West point in the field
     * @return double
     */
    rhoban_geometry::Point field_SW() const;
    /**
     * @brief return the North West point in the field
     * @return double
     */
    rhoban_geometry::Point field_NW() const;
    /**
     * @brief return the South East point in the field
     * @return double
     */
    rhoban_geometry::Point field_NE() const;
    /**
     * @brief return the South East point in the field
     * @return double
     */
    rhoban_geometry::Point field_SE() const;
    /**
     * @brief return field box
     * @return const Box
     */
    Box field() const;
    /**
     * @brief return ally penalty area
     * @return const Box
     */
    Box ally_penalty_area() const;
    /**
     * @brief return opponent penalty area
     * @return const Box
     */
    Box opponent_penalty_area() const;

};


}

#endif
