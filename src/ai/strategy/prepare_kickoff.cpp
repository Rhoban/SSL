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

#include "prepare_kickoff.h"
#include <robot_behavior/do_nothing.h>
#include <robot_behavior/factory.h>
#include <robot_behavior/consign_follower.h>
#include <math/position.h>
#include <core/collection.h>
#include <debug.h>
#include <core/print_collection.h>

namespace rhoban_ssl
{
namespace strategy
{
Prepare_kickoff::Prepare_kickoff(ai::AiData& ai_data)
  : Strategy(ai_data), strategy_is_active(false), placer_when_kicking(ai_data), placer_when_no_kicking(ai_data)
{
}

const std::string Prepare_kickoff::name = "prepare_kickoff";

int Prepare_kickoff::minRobots() const
{
  return 0;
}
int Prepare_kickoff::maxRobots() const
{
  return -1;
}
GoalieNeed Prepare_kickoff::needsGoalie() const
{
  return GoalieNeed::NO;
}
void Prepare_kickoff::update_starting_positions()
{
  attacking_placement = ai_data_.defaultAttackingKickoffPlacement();
  defending_placement = ai_data_.defaultDefendingKickoffPlacement();
  std::function<std::pair<rhoban_geometry::Point, ContinuousAngle>(const Position&)> cvrt =
      [](const Position& position) {
        return std::pair<rhoban_geometry::Point, ContinuousAngle>(position.linear, position.angular);
      };
  placer_when_kicking.set_starting_positions(map2list(attacking_placement.field_robot_position, cvrt));
  placer_when_kicking.set_starting_position_for_goalie(attacking_placement.goalie_position.linear,
                                                       attacking_placement.goalie_position.angular);

  placer_when_no_kicking.set_starting_positions(map2list(defending_placement.field_robot_position, cvrt));
  placer_when_no_kicking.set_starting_position_for_goalie(defending_placement.goalie_position.linear,
                                                          defending_placement.goalie_position.angular);
}
void Prepare_kickoff::start(double time)
{
  DEBUG("START PREPARE KICKOFF");

  update_starting_positions();

  strategy_is_active = true;

  rhoban_geometry::Point linear_position;
  ContinuousAngle angular_position;
  if (is_kicking)
  {
    placer_when_kicking.set_positions(getPlayerIds(),
                                      list2vector(placer_when_kicking.getStartingPositions(getPlayerIds().size())));
    if (placer_when_kicking.getStartingPositionForGoalie(linear_position, angular_position))
    {
      placer_when_kicking.set_goalie_positions(linear_position, angular_position);
    }
    placer_when_kicking.start(time);
  }
  else
  {
    placer_when_no_kicking.set_positions(
        getPlayerIds(), list2vector(placer_when_no_kicking.getStartingPositions(getPlayerIds().size())));
    if (placer_when_no_kicking.getStartingPositionForGoalie(linear_position, angular_position))
    {
      placer_when_no_kicking.set_goalie_positions(linear_position, angular_position);
    }
    placer_when_no_kicking.start(time);
  }
}

void Prepare_kickoff::stop(double time)
{
  if (is_kicking)
  {
    placer_when_kicking.stop(time);
  }
  else
  {
    placer_when_no_kicking.stop(time);
  }
  strategy_is_active = false;
  DEBUG("STOP PREPARE KICKOFF");
}

void Prepare_kickoff::update(double time)
{
  if (is_kicking)
  {
    placer_when_kicking.update(time);
  }
  else
  {
    placer_when_no_kicking.update(time);
  }
}

void Prepare_kickoff::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (is_kicking)
  {
    placer_when_kicking.assignBehaviorToRobots(assign_behavior, time, dt);
  }
  else
  {
    placer_when_no_kicking.assignBehaviorToRobots(assign_behavior, time, dt);
  }
}

Prepare_kickoff::~Prepare_kickoff()
{
}

void Prepare_kickoff::set_kicking(bool value)
{
  assert(not(strategy_is_active));  // You don't have to call set_kicking when the strategy is active. Wait to stop the
                                    // strategy befor usig this function.
  is_kicking = value;
}

std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Prepare_kickoff::getStartingPositions(int number_of_avalaible_robots)
{
  if (is_kicking)
  {
    return placer_when_kicking.getStartingPositions(number_of_avalaible_robots);
  }
  else
  {
    return placer_when_no_kicking.getStartingPositions(number_of_avalaible_robots);
  }
}

bool Prepare_kickoff::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                                       ContinuousAngle& angular_position)
{
  if (is_kicking)
  {
    return placer_when_kicking.getStartingPositionForGoalie(linear_position, angular_position);
  }
  else
  {
    return placer_when_no_kicking.getStartingPositionForGoalie(linear_position, angular_position);
  }
}

void Prepare_kickoff::setRobotAffectation(const std::vector<int>& robot_ids)
{
  Strategy::Strategy::setRobotAffectation(robot_ids);
  if (is_kicking)
  {
    placer_when_kicking.setRobotAffectation(robot_ids);
  }
  else
  {
    placer_when_no_kicking.setRobotAffectation(robot_ids);
  }
}

void Prepare_kickoff::setGoalie(int id, bool to_be_managed)
{
  Strategy::setGoalie(id, to_be_managed);
  if (is_kicking)
  {
    placer_when_kicking.setGoalie(id, to_be_managed);
  }
  else
  {
    placer_when_no_kicking.setGoalie(id, to_be_managed);
  }
}

rhoban_ssl::annotations::Annotations Prepare_kickoff::getAnnotations() const
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

}  // namespace Strategy
}  // namespace rhoban_ssl
