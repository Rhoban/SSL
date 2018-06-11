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

#include "mur.h"

#include <robot_behavior/goalie.h>
#include <robot_behavior/striker.h>

namespace RhobanSSL {
namespace Strategy {

Mur::Mur(Ai::AiData & ai_data):
    Strategy(ai_data)
{
}
    
Mur::~Mur(){
}        

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */ 
int Mur::min_robots() const {
    return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */ 
int Mur::max_robots() const {
    return 2;
}

Goalie_need Mur::needs_goalie() const {
    return Goalie_need::YES;
}

const std::string Mur::name = "mur";

void Mur::start(double time){
    DEBUG("START PREPARE KICKOFF");
    behaviors_are_assigned = false;
}
void Mur::stop(double time){
    DEBUG("STOP PREPARE KICKOFF");
}

void Mur::update(double time){
    const std::vector<int> & players = get_player_ids();
    //int nb_robots = players.size();
    //for( int robot_id : players){
    //	const Ai::Robot & robot = get_robot( robot_id );
    //}
}

void Mur::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( not(behaviors_are_assigned) ){
        // We first assign the behhavior of the goalie.
        assign_behavior(
            get_goalie(), std::shared_ptr<Robot_behavior::RobotBehavior>(
                new Robot_behavior::Goalie(ai_data)
            )
        );

        //we assign now all the other behavior 
        assert( get_player_ids().size() == 1 );
        int id = player_id(0); // we get the first if in get_player_ids()
        assign_behavior(
            id, std::shared_ptr<Robot_behavior::RobotBehavior>(
                new Robot_behavior::Striker(ai_data)
            )
        );

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
> Mur::get_starting_positions( int number_of_avalaible_robots ){
    assert( min_robots() <= number_of_avalaible_robots );
    assert(
        max_robots()==-1 or
        number_of_avalaible_robots <= max_robots() 
    );

    return {
        std::pair<rhoban_geometry::Point,ContinuousAngle>(
            ai_data.relative2absolute(-1.0/3.0, 0.0),
            0.0
        )
    };
}

//
// This function return false if you don't want to 
// give a staring position. So the manager will chose 
// a default position for you.
// 
bool Mur::get_starting_position_for_goalie(
    rhoban_geometry::Point & linear_position, 
    ContinuousAngle & angular_position
){
    linear_position =  ally_goal_center();
    angular_position = ContinuousAngle(0.0);
    return true;
} 



}
}
