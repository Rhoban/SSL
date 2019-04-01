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
PrepareKickoff::PrepareKickoff(ai::AiData& ai_data)
  : Strategy(ai_data), strategy_is_active_(false), placer_when_kicking_(ai_data), placer_when_no_kicking_(ai_data)
{
}

const std::string PrepareKickoff::name = "prepare_kickoff";

int PrepareKickoff::minRobots() const
{
  return 0;
}
int PrepareKickoff::maxRobots() const
{
  return -1;
}
GoalieNeed PrepareKickoff::needsGoalie() const
{
  return GoalieNeed::NO;
}
void PrepareKickoff::updateStartingPositions()
{
  attacking_placement_ = ai_data_.defaultAttackingKickoffPlacement();
  defending_placement_ = ai_data_.defaultDefendingKickoffPlacement();
  std::function<std::pair<rhoban_geometry::Point, ContinuousAngle>(const Position&)> cvrt =
      [](const Position& position) {
        return std::pair<rhoban_geometry::Point, ContinuousAngle>(position.linear, position.angular);
      };
  placer_when_kicking_.setStartingPositions(map2list(attacking_placement_.field_robot_position, cvrt));
  placer_when_kicking_.setStartingPositionForGoalie(attacking_placement_.goalie_position.linear,
                                                       attacking_placement_.goalie_position.angular);

  placer_when_no_kicking_.setStartingPositions(map2list(defending_placement_.field_robot_position, cvrt));
  placer_when_no_kicking_.setStartingPositionForGoalie(defending_placement_.goalie_position.linear,
                                                          defending_placement_.goalie_position.angular);
}
void PrepareKickoff::start(double time)
{
  DEBUG("START PREPARE KICKOFF");

  updateStartingPositions();

  strategy_is_active_ = true;

  rhoban_geometry::Point linear_position;
  ContinuousAngle angular_position;
  if (is_kicking_)
  {
    placer_when_kicking_.setPositions(getPlayerIds(),
                                      list2vector(placer_when_kicking_.getStartingPositions(getPlayerIds().size())));
    if (placer_when_kicking_.getStartingPositionForGoalie(linear_position, angular_position))
    {
      placer_when_kicking_.setGoaliePositions(linear_position, angular_position);
    }
    placer_when_kicking_.start(time);
  }
  else
  {
    placer_when_no_kicking_.setPositions(
        getPlayerIds(), list2vector(placer_when_no_kicking_.getStartingPositions(getPlayerIds().size())));
    if (placer_when_no_kicking_.getStartingPositionForGoalie(linear_position, angular_position))
    {
      placer_when_no_kicking_.setGoaliePositions(linear_position, angular_position);
    }
    placer_when_no_kicking_.start(time);
  }
}

void PrepareKickoff::stop(double time)
{
  if (is_kicking_)
  {
    placer_when_kicking_.stop(time);
  }
  else
  {
    placer_when_no_kicking_.stop(time);
  }
  strategy_is_active_ = false;
  DEBUG("STOP PREPARE KICKOFF");
}

void PrepareKickoff::update(double time)
{
  if (is_kicking_)
  {
    placer_when_kicking_.update(time);
  }
  else
  {
    placer_when_no_kicking_.update(time);
  }
}

void PrepareKickoff::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (is_kicking_)
  {
    placer_when_kicking_.assignBehaviorToRobots(assign_behavior, time, dt);
  }
  else
  {
    placer_when_no_kicking_.assignBehaviorToRobots(assign_behavior, time, dt);
  }
}

PrepareKickoff::~PrepareKickoff()
{
}

void PrepareKickoff::setKicking(bool value)
{
  assert(not(strategy_is_active_));  // You don't have to call set_kicking when the strategy is active. Wait to stop the
                                    // strategy befor usig this function.
  is_kicking_ = value;
}

std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
PrepareKickoff::getStartingPositions(int number_of_avalaible_robots)
{
  if (is_kicking_)
  {
    return placer_when_kicking_.getStartingPositions(number_of_avalaible_robots);
  }
  else
  {
    return placer_when_no_kicking_.getStartingPositions(number_of_avalaible_robots);
  }
}

bool PrepareKickoff::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                                       ContinuousAngle& angular_position)
{
  if (is_kicking_)
  {
    return placer_when_kicking_.getStartingPositionForGoalie(linear_position, angular_position);
  }
  else
  {
    return placer_when_no_kicking_.getStartingPositionForGoalie(linear_position, angular_position);
  }
}

void PrepareKickoff::setRobotAffectation(const std::vector<int>& robot_ids)
{
  Strategy::Strategy::setRobotAffectation(robot_ids);
  if (is_kicking_)
  {
    placer_when_kicking_.setRobotAffectation(robot_ids);
  }
  else
  {
    placer_when_no_kicking_.setRobotAffectation(robot_ids);
  }
}

void PrepareKickoff::setGoalie(int id, bool to_be_managed)
{
  Strategy::setGoalie(id, to_be_managed);
  if (is_kicking_)
  {
    placer_when_kicking_.setGoalie(id, to_be_managed);
  }
  else
  {
    placer_when_no_kicking_.setGoalie(id, to_be_managed);
  }
}

rhoban_ssl::annotations::Annotations PrepareKickoff::getAnnotations() const
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
