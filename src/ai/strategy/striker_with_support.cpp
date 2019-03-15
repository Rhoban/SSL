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

#include "striker_with_support.h"

#include <robot_behavior/goalie.h>
#include <robot_behavior/striker.h>
#include <robot_behavior/robot_follower.h>

namespace RhobanSSL
{
namespace Strategy
{
StrikerWithSupport::StrikerWithSupport(Ai::AiData& ai_data) : Strategy(ai_data)
{
}

StrikerWithSupport::~StrikerWithSupport()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int StrikerWithSupport::min_robots() const
{
  return 3;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int StrikerWithSupport::max_robots() const
{
  return 3;
}

Goalie_need StrikerWithSupport::needs_goalie() const
{
  return Goalie_need::NO;
}

const std::string StrikerWithSupport::name = "striker_with_support";

void StrikerWithSupport::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned = false;

  striker = std::shared_ptr<Robot_behavior::Striker>(new Robot_behavior::Striker(ai_data));
}
void StrikerWithSupport::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void StrikerWithSupport::update(double time)
{
  // std::pair<rhoban_geometry::Point, double> results = GameInformations::find_goal_best_move( ball_position() );
  // rhoban_geometry::Point goal_point = results.first;

  // static_cast<Robot_behavior::Striker*>( striker.get() )->declare_point_to_strik( goal_point );
}

void StrikerWithSupport::assign_behavior_to_robots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (not(behaviors_are_assigned))
  {
    // we assign now all the other behavior
    assert(get_player_ids().size() == 3);

    assign_behavior(player_id(0), striker);
    int supportLeft = player_id(1);  // we get the first if in get_player_ids()
    std::shared_ptr<Robot_behavior::RobotFollower> support_behaviorL(new Robot_behavior::RobotFollower(ai_data));
    support_behaviorL->declare_robot_to_follow(player_id(0), Vector2d(1, 0.5), Vision::Team::Ally);
    assign_behavior(supportLeft, support_behaviorL);

    int supportRight = player_id(2);  // we get the first if in get_player_ids()
    std::shared_ptr<Robot_behavior::RobotFollower> support_behaviorR(new Robot_behavior::RobotFollower(ai_data));
    support_behaviorR->declare_robot_to_follow(player_id(0), Vector2d(1, -0.5), Vision::Team::Ally);
    assign_behavior(supportRight, support_behaviorR);

    behaviors_are_assigned = true;
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
StrikerWithSupport::get_starting_positions(int number_of_avalaible_robots)
{
  assert(min_robots() <= number_of_avalaible_robots);
  assert(max_robots() == -1 or number_of_avalaible_robots <= max_robots());

  return {
    // std::pair<rhoban_geometry::Point,ContinuousAngle>(
    //     ai_data.relative2absolute(-1.0/3.0, 0.0),
    //     0.0
    // )
  };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool StrikerWithSupport::get_starting_position_for_goalie(rhoban_geometry::Point& linear_position,
                                                          ContinuousAngle& angular_position)
{
  linear_position = ally_goal_center();
  angular_position = ContinuousAngle(0.0);
  return true;
}

RhobanSSLAnnotation::Annotations StrikerWithSupport::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;

  for (auto it = this->get_player_ids().begin(); it != this->get_player_ids().end(); it++)
  {
    const rhoban_geometry::Point& robot_position = get_robot(*it).get_movement().linear_position(time());
    // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
    annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
  }
  return annotations;
}

}  // namespace Strategy
}  // namespace RhobanSSL
