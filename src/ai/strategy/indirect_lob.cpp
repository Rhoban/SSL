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
namespace strategy
{
IndirectLob::IndirectLob(ai::AiData& ai_data) : Strategy(ai_data), state_(0)
{
}

IndirectLob::~IndirectLob()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int IndirectLob::minRobots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int IndirectLob::maxRobots() const
{
  return 2;
}

GoalieNeed IndirectLob::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string IndirectLob::name = "indirect_lob";

void IndirectLob::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned_ = false;
}
void IndirectLob::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void IndirectLob::update(double time)
{
  int seuil = 0.4;
  int pass = playerId(1);  // we get the first if in get_player_ids()
  const ai::Robot& robot_pass = getRobot(pass);
  const rhoban_geometry::Point& robot_pass_position = robot_pass.getMovement().linearPosition(time);

  Vector2d ball_robot_vector_pass = ballPosition() - robot_pass_position;

  std::cout << "yeeeeeah " << state_ << "mpyteozur " << ball_robot_vector_pass.norm() << '\n';
  if (ball_robot_vector_pass.norm() <= seuil)
  {
    state_ = 1;
  }
}

void IndirectLob::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (not(behaviors_are_assigned_))
  {
    // we assign now all the other behavior
    assert(getPlayerIds().size() == 2);

    int wait_pass = playerId(0);  // we get the first if in get_player_ids()
    int pass = playerId(1);       // we get the first if in get_player_ids()

    if (state_ == 0)
    {
      assign_behavior(wait_pass,
                      std::shared_ptr<robot_behavior::SearchShootArea>(new robot_behavior::SearchShootArea(ai_data_)));

      std::shared_ptr<robot_behavior::Pass> pass_behavior(new robot_behavior::Pass(ai_data_));
      pass_behavior->declareRobotToPass(wait_pass, vision::Ally);
      assign_behavior(pass, pass_behavior);
      std::cout << "stat aaaaaaaaaa " << state_ << '\n';
    }
    else
    {
      assign_behavior(wait_pass, std::shared_ptr<robot_behavior::Striker>(new robot_behavior::Striker(ai_data_)));

      std::shared_ptr<robot_behavior::RobotFollower> support(new robot_behavior::RobotFollower(ai_data_));
      support->declare_robot_to_follow_(wait_pass, Vector2d(0.5, 0.0), vision::Ally);
      assign_behavior(pass, support);
      std::cout << "stat bbbbbbbb " << state_ << '\n';
    }

    behaviors_are_assigned_ = true;
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
IndirectLob::getStartingPositions(int number_of_avalaible_robots)
{
  assert(minRobots() <= number_of_avalaible_robots);
  assert(maxRobots() == -1 or number_of_avalaible_robots <= maxRobots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(ballPosition(), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool IndirectLob::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                               ContinuousAngle& angular_position)
{
  linear_position = allyGoalCenter();
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations IndirectLob::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;

  for (auto it = this->getPlayerIds().begin(); it != this->getPlayerIds().end(); it++)
  {
    const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linearPosition(time());
    // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
    annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
  }
  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl
