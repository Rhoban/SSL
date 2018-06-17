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

#include "attaque_with_support.h"


namespace RhobanSSL {
namespace Strategy {

AttaqueWithSupport::AttaqueWithSupport(Ai::AiData & ai_data):
    Strategy(ai_data)
{
}

AttaqueWithSupport::~AttaqueWithSupport(){
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int AttaqueWithSupport::min_robots() const {
    return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int AttaqueWithSupport::max_robots() const {
    return 2;
}

Goalie_need AttaqueWithSupport::needs_goalie() const {
    return Goalie_need::NO;
}

const std::string AttaqueWithSupport::name = "attaque_with_support";

void AttaqueWithSupport::start(double time){
    DEBUG("START PREPARE KICKOFF");
    behaviors_are_assigned = false;
}
void AttaqueWithSupport::stop(double time){
    DEBUG("STOP PREPARE KICKOFF");
}

void AttaqueWithSupport::update(double time){
}

void AttaqueWithSupport::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( not(behaviors_are_assigned) ){
        //we assign now all the other behavior
        assert( get_player_ids().size() == 2 );
        int strikerID = player_id(1); // we get the first if in get_player_ids()
        striker = std::shared_ptr<Robot_behavior::Striker>(
            new Robot_behavior::Striker(ai_data)
        );
        assign_behavior( strikerID, striker );

        int supportID = player_id(0);
        support = std::shared_ptr<Robot_behavior::RobotFollower>(
            new Robot_behavior::RobotFollower(ai_data)
        );
        support->declare_robot_to_follow(strikerID, Vector2d(1.0, 1.0), Vision::Team::Ally);
        assign_behavior( supportID, support );

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
> AttaqueWithSupport::get_starting_positions( int number_of_avalaible_robots ){
    assert( min_robots() <= number_of_avalaible_robots );
    assert(
        max_robots()==-1 or
        number_of_avalaible_robots <= max_robots()
    );

    return {
        std::pair<rhoban_geometry::Point,ContinuousAngle>(
            ball_position(),
            0.0
        )
    };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool AttaqueWithSupport::get_starting_position_for_goalie(
    rhoban_geometry::Point & linear_position,
    ContinuousAngle & angular_position
){
    linear_position =  ally_goal_center();
    angular_position = ContinuousAngle(0.0);
    return true;
}



}
}
