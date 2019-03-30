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

#include "indirect_lob.h"

#include <robot_behavior/pass.h>
#include <robot_behavior/search_shoot_area.h>
#include <robot_behavior/striker.h>
#include <robot_behavior/robot_follower.h>

namespace rhoban_ssl
{
namespace Strategy
{
IndirectLob::IndirectLob(Ai::AiData& ai_data) : Strategy(ai_data), state(0)
{
}

IndirectLob::~IndirectLob()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int IndirectLob::min_robots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int IndirectLob::max_robots() const
{
  return 2;
}

Goalie_need IndirectLob::needs_goalie() const
{
  return Goalie_need::NO;
}

const std::string IndirectLob::name = "indirect_lob";

void IndirectLob::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned = false;
}
void IndirectLob::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void IndirectLob::update(double time)
{
  int seuil = 0.4;
  int pass = player_id(1);  // we get the first if in get_player_ids()
  const Ai::Robot& robot_pass = getRobot(pass);
  const rhoban_geometry::Point& robot_pass_position = robot_pass.get_movement().linear_position(time);

  Vector2d ball_robot_vector_pass = ballPosition() - robot_pass_position;

  std::cout << "yeeeeeah " << state << "mpyteozur " << ball_robot_vector_pass.norm() << '\n';
  if (ball_robot_vector_pass.norm() <= seuil)
  {
    state = 1;
  }
}

void IndirectLob::assign_behavior_to_robots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (not(behaviors_are_assigned))
  {
    // we assign now all the other behavior
    assert(get_player_ids().size() == 2);

    int wait_pass = player_id(0);  // we get the first if in get_player_ids()
    int pass = player_id(1);       // we get the first if in get_player_ids()

    if (state == 0)
    {
      assign_behavior(wait_pass,
                      std::shared_ptr<Robot_behavior::SearchShootArea>(new Robot_behavior::SearchShootArea(ai_data)));

      std::shared_ptr<Robot_behavior::Pass> pass_behavior(new Robot_behavior::Pass(ai_data));
      pass_behavior->declare_robot_to_pass(wait_pass, Vision::Team::Ally);
      assign_behavior(pass, pass_behavior);
      std::cout << "stat aaaaaaaaaa " << state << '\n';
    }
    else
    {
      assign_behavior(wait_pass, std::shared_ptr<Robot_behavior::Striker>(new Robot_behavior::Striker(ai_data)));

      std::shared_ptr<Robot_behavior::RobotFollower> support(new Robot_behavior::RobotFollower(ai_data));
      support->declare_robot_to_follow(wait_pass, Vector2d(0.5, 0.0), Vision::Team::Ally);
      assign_behavior(pass, support);
      std::cout << "stat bbbbbbbb " << state << '\n';
    }

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
IndirectLob::get_starting_positions(int number_of_avalaible_robots)
{
  assert(min_robots() <= number_of_avalaible_robots);
  assert(max_robots() == -1 or number_of_avalaible_robots <= max_robots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(ballPosition(), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool IndirectLob::get_starting_position_for_goalie(rhoban_geometry::Point& linear_position,
                                                   ContinuousAngle& angular_position)
{
  linear_position = allyGoalCenter();
  angular_position = ContinuousAngle(0.0);
  return true;
}

RhobanSSLAnnotation::Annotations IndirectLob::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;

  for (auto it = this->get_player_ids().begin(); it != this->get_player_ids().end(); it++)
  {
    const rhoban_geometry::Point& robot_position = getRobot(*it).get_movement().linear_position(time());
    // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
    annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
  }
  return annotations;
}

}  // namespace Strategy
}  // namespace rhoban_ssl
