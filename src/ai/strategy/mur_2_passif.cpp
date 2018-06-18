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

#include "mur_2_passif.h"

#include <robot_behavior/goalie.h>
#include <robot_behavior/striker.h>
#include <robot_behavior/mur_defensor.h>

namespace RhobanSSL {
namespace Strategy {

Mur_2_passif::Mur_2_passif(Ai::AiData & ai_data):
    Strategy(ai_data)
{
}

Mur_2_passif::~Mur_2_passif(){
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Mur_2_passif::min_robots() const {
    return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Mur_2_passif::max_robots() const {
    return 2;
}

Goalie_need Mur_2_passif::needs_goalie() const {
    return Goalie_need::NO;
}

const std::string Mur_2_passif::name = "Mur_2_passif";

void Mur_2_passif::start(double time){
    DEBUG("START PREPARE KICKOFF");
    behaviors_are_assigned = false;
}
void Mur_2_passif::stop(double time){
    DEBUG("STOP PREPARE KICKOFF");
}

void Mur_2_passif::update(double time){

  int nearest_ally_robot_from_ball = GameInformations::get_nearest_ball( );
  is_closest_0 = false;
  is_closest_1 = false;

  if ( nearest_ally_robot_from_ball == player_id(0) ){
      is_closest_0 = true;
  } else {
      if (nearest_ally_robot_from_ball == player_id(1)) {
        is_closest_1 = true;
      }
  }

}

void Mur_2_passif::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){

    std::shared_ptr<Robot_behavior::RobotBehavior> mur1(
            new Robot_behavior::Mur_defensor(ai_data, 1)
    );
    static_cast<Robot_behavior::Mur_defensor*>( mur1.get() )->declare_mur_robot_id( 0, 2 );

    std::shared_ptr<Robot_behavior::RobotBehavior> mur2(
            new Robot_behavior::Mur_defensor(ai_data, 1)
    );

    if( not(behaviors_are_assigned) ){

        assert( get_player_ids().size() == 2 );

        assign_behavior( player_id(0), mur1 );
        assign_behavior( player_id(1), mur2 );

        behaviors_are_assigned = true;
    }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<
    std::pair<rhoban_geometry::Point,ContinuousAngle>
> Mur_2_passif::get_starting_positions( int number_of_avalaible_robots ){
    assert( min_robots() <= number_of_avalaible_robots );
    assert(
        max_robots()==-1 or
        number_of_avalaible_robots <= max_robots()
    );

    return {
        std::pair<rhoban_geometry::Point,ContinuousAngle>(
            ally_goal_center(),
            0.0
        )
    };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool Mur_2_passif::get_starting_position_for_goalie(
    rhoban_geometry::Point & linear_position,
    ContinuousAngle & angular_position
){
    linear_position =  ally_goal_center();
    angular_position = ContinuousAngle(0.0);
    return true;
}



}
}
