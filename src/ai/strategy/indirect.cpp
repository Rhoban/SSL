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

#include "indirect.h"
#include <debug.h>
#include <robot_behavior/pass.h>
#include <robot_behavior/search_shoot_area.h>
#include <robot_behavior/striker.h>
#include <robot_behavior/robot_follower.h>

namespace RhobanSSL {
namespace Strategy {

Indirect::Indirect(Ai::AiData & ai_data):
    Strategy(ai_data),
    state(0)
{
}

Indirect::~Indirect(){
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Indirect::min_robots() const {
    return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Indirect::max_robots() const {
    return 2;
}

Goalie_need Indirect::needs_goalie() const {
    return Goalie_need::NO;
}

const std::string Indirect::name = "indirect";

void Indirect::start(double time){
    DEBUG("START PREPARE KICKOFF");
    behaviors_are_assigned = false;
}
void Indirect::stop(double time){
    DEBUG("STOP PREPARE KICKOFF");
}

void Indirect::update(double time){
}

void Indirect::assign_behavior_to_robots(
  std::function<
  void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
  > assign_behavior,
  double time, double dt
){
  assert( get_player_ids().size() == 2 );

  int wait_pass = player_id(0); // we get the first if in get_player_ids()
  int pass = player_id(1); // we get the first if in get_player_ids()
  double seuil = 0.2;
  const Ai::Robot & robot_pass = get_robot( pass );
  const rhoban_geometry::Point & robot_pass_position = robot_pass.get_movement().linear_position( time );

  // Vector2d ball_robot_vector_pass = ball_position() - robot_pass_position;
  // double d = ball_robot_vector_pass.norm();
  // if( d <= seuil){
  //   state = 1;
  // }

  if( state == 0){
    // DEBUG("STATE "  << state);
    if (not(behaviors_are_assigned)) {
        assign_behavior(
            wait_pass, std::shared_ptr<Robot_behavior::SearchShootArea>(
              new Robot_behavior::SearchShootArea(ai_data)
            )
        );
        pass_behavior = std::shared_ptr<Robot_behavior::Pass_dribbler>(
            new Robot_behavior::Pass_dribbler(ai_data)
        );
        pass_behavior->declare_point_to_pass(get_robot(wait_pass, Vision::Team::Ally).get_movement().linear_position( time ));
        assign_behavior( pass, pass_behavior );
    }

    // DEBUG("NEED TO KICK before " << pass_behavior->need_to_kick);
    assert( pass_behavior.get() );
    if (pass_behavior->need_to_kick) {
      // DEBUG("NEED TO KICK");
      state = 1;
    }

  }else if( state == 1 ){
    // DEBUG("STATE "  << state);
    assign_behavior(
      wait_pass, std::shared_ptr<Robot_behavior::Striker>(
        new Robot_behavior::Striker(ai_data)
      )
    );

    std::shared_ptr<Robot_behavior::RobotFollower> support(
      new Robot_behavior::RobotFollower(ai_data)
    );
    support->declare_robot_to_follow(wait_pass, Vector2d(0.5, 0.0), Vision::Team::Ally);
    assign_behavior( pass, support );
  }



  behaviors_are_assigned = true;
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<
    std::pair<rhoban_geometry::Point,ContinuousAngle>
> Indirect::get_starting_positions( int number_of_avalaible_robots ){
    assert( min_robots() <= number_of_avalaible_robots );
    assert(
        max_robots()==-1 or
        number_of_avalaible_robots <= max_robots()
    );

    return {
        std::pair<rhoban_geometry::Point,ContinuousAngle>(
            ball_position(),
            0.0
        ),
        std::pair<rhoban_geometry::Point,ContinuousAngle>(
            oponent_goal_center(),
            0.0
        )
    };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool Indirect::get_starting_position_for_goalie(
    rhoban_geometry::Point & linear_position,
    ContinuousAngle & angular_position
){
    linear_position =  ally_goal_center();
    angular_position = ContinuousAngle(0.0);
    return true;
}



}
}
