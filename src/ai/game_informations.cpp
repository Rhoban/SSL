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
#include "math/lines.h"

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

rhoban_geometry::Point GameInformations::opponent_goal_center() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, 0.0 );
}

rhoban_geometry::Point GameInformations::center_mark() const {
    return rhoban_geometry::Point( 0.0, 0.0 );
}

rhoban_geometry::Point GameInformations::opponent_corner_right() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, - ai_data.field.fieldWidth/2.0 );
}

rhoban_geometry::Point GameInformations::opponent_corner_left() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, ai_data.field.fieldWidth/2.0 );
}

const Ai::Robot & GameInformations::get_robot( int robot_number, Vision::Team team ) const {
    return ai_data.robots.at(team).at(robot_number);
}


void GameInformations::get_robot_in_line(
    const rhoban_geometry::Point p1, const rhoban_geometry::Point p2, Vision::Team team, double distance,
    std::vector<int> & result
) const{
    if(norm_square(p1 - p2) == 0) {
        return;
    }

    for (size_t i = 0; i < Ai::Constants::NB_OF_ROBOTS_BY_TEAM; i++) {
        const Ai::Robot& robot = get_robot(i, team);
        if (robot.is_present_in_vision()) {
            const rhoban_geometry::Point& robot_position =
                robot.get_movement().linear_position(time());
            if (distance_from_point_to_line(robot_position, p1, p2) <=
                distance) {
                result.push_back(i);
            }
        }
    }
}

std::vector<int> GameInformations::get_robot_in_line(
    const rhoban_geometry::Point p1, const rhoban_geometry::Point p2, Vision::Team team, double distance
) const{
    std::vector<int> result;
    get_robot_in_line(p1, p2, team, distance, result);
    return result;
}

std::vector<int> GameInformations::get_robot_in_line( const rhoban_geometry::Point p1, const rhoban_geometry::Point p2, double distance ) const{
    std::vector<int> result;
    get_robot_in_line(p1, p2, Vision::Team::Ally, distance, result);
    get_robot_in_line(p1, p2, Vision::Team::Opponent, distance, result);
    return result;
}



std::pair<rhoban_geometry::Point, double> GameInformations::find_goal_best_move( const rhoban_geometry::Point point, const rhoban_geometry::Point goal ) const {

    rhoban_geometry::Point opponent_goal_point;
    if ( goal == rhoban_geometry::Point(66,66) ) {
        opponent_goal_point = opponent_goal_center();
    } else {
        opponent_goal_point = goal;
    }

    rhoban_geometry::Point return_point;
    const rhoban_geometry::Point left_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0, ai_data.field.goalWidth / 2.0 );
    const rhoban_geometry::Point right_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0 );
    const Vector2d left_right_post_vector = right_post_position - left_post_position;
    const double dist_post = left_right_post_vector.norm();
    const int nb_analysed_point = 16;
    rhoban_geometry::Point analysed_point;

    int nb_valid_path = 0;
    double max_valid_path = 0.0;
    int max_i = 0;
    bool max_valid_combo_begin = false;

    for (size_t i = 1; i < nb_analysed_point - 1; i++) {
        analysed_point = right_post_position + rhoban_geometry::Point(0, dist_post / nb_analysed_point * i);
        std::vector<int> robot_in_line = GameInformations::get_robot_in_line( point, analysed_point,Vision::Team::Opponent, 0.15 );
        std::vector<int> robot_in_line2 = GameInformations::get_robot_in_line( point, analysed_point,Vision::Team::Ally, 0.15 );
        robot_in_line.insert( robot_in_line.end(), robot_in_line2.begin(), robot_in_line2.end() );
        if (robot_in_line.empty()) {
            nb_valid_path++;
            if (nb_valid_path > max_valid_path) {
                max_valid_path = nb_valid_path;
                if (max_valid_combo_begin == false) {
                    max_valid_combo_begin = true;
                    max_i = i;
                }
            }
        } else {
            nb_valid_path = 0;
            max_valid_combo_begin = false;
        }
    }

    if (max_valid_path == 0) {
        return_point = opponent_goal_point;
    } else {
        return_point = right_post_position + rhoban_geometry::Point(0, dist_post / nb_analysed_point * (max_i + max_valid_path / 2));
    }

    double proba = max_valid_path / nb_analysed_point;

    std::pair<rhoban_geometry::Point, double> results(return_point, proba);

    return results;
}

int GameInformations::get_shirt_number_of_closest_robot_to_the_ball( Vision::Team team ) const{
    return get_shirt_number_of_closest_robot(team, ball_position());
}


int GameInformations::get_shirt_number_of_closest_robot(Vision::Team team, rhoban_geometry::Point point) const{
    int id = -1;
    double distance_max = -1;
    for (int i = 0; i < Ai::Constants::NB_OF_ROBOTS_BY_TEAM; i++) {
      const Ai::Robot & robot = get_robot( i,  team );
      if(robot.is_present_in_vision()){
        const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( time() );
        double distance = robot_position.getDist(point);
        if (id == -1 or distance < distance_max) {
          distance_max = distance;
          id = i;
        }
      }
    }
    return id;
}



double GameInformations::get_robot_distance_from_ally_goal_center(int robot_number, Vision::Team team ) const{
  double distance = -1;
  const Ai::Robot & robot = get_robot( robot_number,  team );
  if(robot.is_present_in_vision()){
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( time() );
    Vector2d goal_center_robot = robot_position - ally_goal_center();
    distance = goal_center_robot.norm();
    distance = (ai_data.field.fieldLength - distance)/ai_data.field.fieldLength;
  }
  return distance;
}

std::vector<double> GameInformations::threat( Vision::Team team ) const{
  std::vector<double> v_threat;
  for (size_t i = 0; i < Ai::Constants::NB_OF_ROBOTS_BY_TEAM; i++) {
    double threat = get_robot_distance_from_ally_goal_center(i, team);
    v_threat.push_back(threat);
  }
  return v_threat;
}

int GameInformations::shirt_number_of_threat_max( Vision::Team team ) const{
  int id = -1;
  double threat_max = -1;

  std::vector<double> v_threat = threat( team );
  for (size_t i = 0; i < v_threat.size(); i++) {
    double threat = v_threat[i];
    if (threat > threat_max) {
      threat_max = threat;
      id = i;
    }
  }
  return id;
}

int GameInformations::shirt_number_of_threat_max_2( Vision::Team team ) const{ // second threat max
  int id_1 = -1;
  int id_2 = -1;
  double threat_max = -1;
  double threat_max_2 = -1;

  std::vector<double> v_threat = threat( team );
  for (size_t i = 0; i < v_threat.size(); i++) {
    double threat = v_threat[i];
    if (threat > threat_max) {
      threat_max_2 = threat_max;
      threat_max = threat;
      id_2 = id_1;
      id_1 = i;
    }
    else if( threat > threat_max_2){
      threat_max_2 = threat;
      id_2 = i;
    }
  }
  return id_2;
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

double GameInformations::field_height() const{
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

double GameInformations::penalty_area_height() const {
    return ai_data.field.penaltyAreaDepth;
}

bool GameInformations::infra_red( int robot_number, Vision::Team team ) const{
    return get_robot( robot_number, team ).infra_red;
}

};
