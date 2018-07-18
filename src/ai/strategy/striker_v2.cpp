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

#include "striker_v2.h"

#include <robot_behavior/striker.h>
#include <robot_behavior/mur_defensor.h>
#include <robot_behavior/degageur.h>

namespace RhobanSSL {
namespace Strategy {

StrikerV2::StrikerV2(Ai::AiData & ai_data):
    Strategy(ai_data)
{
}

StrikerV2::~StrikerV2(){
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int StrikerV2::min_robots() const {
    return 1;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int StrikerV2::max_robots() const {
    return 1;
}

Goalie_need StrikerV2::needs_goalie() const {
    return Goalie_need::NO;
}

const std::string StrikerV2::name = "striker_v2";

void StrikerV2::start(double time){
    DEBUG("START PREPARE KICKOFF");
    behaviors_are_assigned = false;

    striker = std::shared_ptr<Robot_behavior::Striker>(
      new Robot_behavior::Striker(ai_data)
    );
}
void StrikerV2::stop(double time){
    DEBUG("STOP PREPARE KICKOFF");
}

void StrikerV2::update(double time){
    std::pair<rhoban_geometry::Point, double> results = GameInformations::find_goal_best_move( ball_position() );
    striker->declare_point_to_strik(results.first);
}

void StrikerV2::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){

    if( not(behaviors_are_assigned) ){

        assert( get_player_ids().size() == 1 );

        assign_behavior( player_id(0), striker );

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
> StrikerV2::get_starting_positions( int number_of_avalaible_robots ){
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
bool StrikerV2::get_starting_position_for_goalie(
    rhoban_geometry::Point & linear_position,
    ContinuousAngle & angular_position
){
    linear_position =  ally_goal_center();
    angular_position = ContinuousAngle(0.0);
    return true;
}

RhobanSSLAnnotation::Annotations StrikerV2::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;

    for (auto it = this->get_player_ids().begin(); it != this->get_player_ids().end(); it++)
    {
        const rhoban_geometry::Point & robot_position = get_robot(*it).get_movement().linear_position( time() );
        //annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
        annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
    }
    return annotations;
}



}
}
