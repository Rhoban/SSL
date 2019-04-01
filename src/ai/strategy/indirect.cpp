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

namespace rhoban_ssl
{
namespace strategy
{
Indirect::Indirect(ai::AiData& ai_data) : Strategy(ai_data), state_(0)
{
}

Indirect::~Indirect()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Indirect::minRobots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Indirect::maxRobots() const
{
  return 2;
}

GoalieNeed Indirect::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string Indirect::name = "indirect";

void Indirect::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned_ = false;
}
void Indirect::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void Indirect::update(double time)
{
}

void Indirect::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  assert(getPlayerIds().size() == 2);

  int wait_pass = playerId(0);  // we get the first if in get_player_ids()
  int pass = playerId(1);       // we get the first if in get_player_ids()
  // double seuil = 0.2;
  // const ai::Robot & robot_pass = get_robot( pass );
  // const rhoban_geometry::Point & robot_pass_position = robot_pass.get_movement().linear_position( time );

  // Vector2d ball_robot_vector_pass = ball_position() - robot_pass_position;
  // double d = ball_robot_vector_pass.norm();
  // if( d <= seuil){
  //   state = 1;
  // }

  if (state_ == 0)
  {
    // DEBUG("STATE "  << state);
    if (not(behaviors_are_assigned_))
    {
      assign_behavior(wait_pass,
                      std::shared_ptr<robot_behavior::SearchShootArea>(new robot_behavior::SearchShootArea(ai_data_)));
      pass_behavior_ = std::shared_ptr<robot_behavior::PassDribbler>(new robot_behavior::PassDribbler(ai_data_));
      pass_behavior_->declarePointToPass(getRobot(wait_pass, vision::Team::Ally).getMovement().linearPosition(time));
      assign_behavior(pass, pass_behavior_);
    }

    // DEBUG("NEED TO KICK before " << pass_behavior->need_to_kick);
    assert(pass_behavior_.get());
    if (pass_behavior_->need_to_kick)
    {
      // DEBUG("NEED TO KICK");
      state_ = 1;
    }
  }
  else if (state_ == 1)
  {
    // DEBUG("STATE "  << state);
    assign_behavior(wait_pass, std::shared_ptr<robot_behavior::Striker>(new robot_behavior::Striker(ai_data_)));

    std::shared_ptr<robot_behavior::RobotFollower> support(new robot_behavior::RobotFollower(ai_data_));
    support->declare_robot_to_follow_(wait_pass, Vector2d(0.5, 0.0), vision::Team::Ally);
    assign_behavior(pass, support);
  }

  behaviors_are_assigned_ = true;
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Indirect::getStartingPositions(int number_of_avalaible_robots)
{
  assert(minRobots() <= number_of_avalaible_robots);
  assert(maxRobots() == -1 or number_of_avalaible_robots <= maxRobots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(ballPosition(), 0.0),
           std::pair<rhoban_geometry::Point, ContinuousAngle>(opponentGoalCenter(), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool Indirect::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position, ContinuousAngle& angular_position)
{
  linear_position = allyGoalCenter();
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations Indirect::getAnnotations() const
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
