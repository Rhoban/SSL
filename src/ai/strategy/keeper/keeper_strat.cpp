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

#include "keeper_strat.h"

namespace rhoban_ssl
{
namespace strategy
{
KeeperStrat::KeeperStrat()
  : Strategy(), clearer_(std::shared_ptr<robot_behavior::keeper::Clearer>(new robot_behavior::keeper::Clearer()))
{
}

KeeperStrat::~KeeperStrat()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int KeeperStrat::minRobots() const
{
  return 0;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int KeeperStrat::maxRobots() const
{
  return 0;
}

GoalieNeed KeeperStrat::needsGoalie() const
{
  return GoalieNeed::YES;
}

const std::string KeeperStrat::name = "goalie_strat";

void KeeperStrat::start(double time)
{
  DEBUG("START KEEPER STRAT");
  goalie_ = std::shared_ptr<robot_behavior::Keeper>(new robot_behavior::Keeper());
  behaviors_are_assigned_ = false;
}
void KeeperStrat::stop(double time)
{
  DEBUG("STOP KEEPER STRAT");
}

void KeeperStrat::update(double time)
{
}

void KeeperStrat::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  // we assign now all the other behavior

  int goalieID = getGoalie();  // we get the first if in get_player_ids()

  if (allyPenaltyArea().is_inside(ballPosition()))
  {
    assign_behavior(goalieID, clearer_);
  }
  else
  {
    assign_behavior(goalieID, goalie_);
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
KeeperStrat::getStartingPositions(int number_of_avalaible_robots)
{
  assert(minRobots() <= number_of_avalaible_robots);
  assert(maxRobots() == -1 or number_of_avalaible_robots <= maxRobots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(Data::get()->field.goalCenter(Ally), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool KeeperStrat::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                               ContinuousAngle& angular_position)
{
  linear_position = Data::get()->field.goalCenter(Ally);
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations KeeperStrat::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  /*
    for (auto it = this->getPlayerIds().begin(); it != this->getPlayerIds().end(); it++)
    {
      const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linearPosition(time());
      // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
      annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30,
    "white");
    }*/
  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl
