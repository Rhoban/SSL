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

#include "union.h"

namespace rhoban_ssl
{
namespace strategy
{
Union::Union(ai::AiData& ai_data) : Strategy(ai_data), min(0), max(0)
{
}

void Union::add_goalie_strategy(std::shared_ptr<Strategy> strategy)
{
  min += strategy->minRobots();
  strategy_with_goal = strategy;
}

void Union::add_strategy(std::shared_ptr<Strategy> strategy)
{
  if (strategy->maxRobots() < 0)
  {
    max = -1;
  }
  else
  {
    if (max >= 0)
    {
      max += strategy->maxRobots();
    }
  }
  strategies_without_goal.push_back(strategy);
}

int Union::minRobots() const
{
  return min;
}
int Union::maxRobots() const
{
  return max;
}
GoalieNeed Union::needsGoalie() const
{
  if (strategy_with_goal)
  {
    return GoalieNeed::YES;
  }
  else
  {
    return GoalieNeed::NO;
  }
};

Union::~Union()
{
}

void Union::clear()
{
  strategies_without_goal.clear();
  strategy_with_goal.reset();
  min = 0;
  max = 0;
}

void Union::update(double time)
{
  if (strategy_with_goal)
  {
    strategy_with_goal->update(time);
  }
  for (std::shared_ptr<Strategy>& elem : strategies_without_goal)
  {
    elem->update(time);
  }
}

void Union::start(double time)
{
  if (strategy_with_goal)
  {
    strategy_with_goal->start(time);
  }
  for (std::shared_ptr<Strategy>& elem : strategies_without_goal)
  {
    elem->start(time);
  }
}

void Union::stop(double time)
{
  if (strategy_with_goal)
  {
    strategy_with_goal->stop(time);
  }
  for (std::shared_ptr<Strategy>& elem : strategies_without_goal)
  {
    elem->stop(time);
  }
}

void Union::pause(double time)
{
  if (strategy_with_goal)
  {
    strategy_with_goal->pause(time);
  }
  for (std::shared_ptr<Strategy>& elem : strategies_without_goal)
  {
    elem->pause(time);
  }
}

void Union::resume(double time)
{
  if (strategy_with_goal)
  {
    strategy_with_goal->resume(time);
  }
  for (std::shared_ptr<Strategy>& elem : strategies_without_goal)
  {
    elem->resume(time);
  }
}

void Union::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (strategy_with_goal)
  {
    strategy_with_goal->assignBehaviorToRobots(assign_behavior, time, dt);
  }
  for (std::shared_ptr<Strategy>& elem : strategies_without_goal)
  {
    elem->assignBehaviorToRobots(assign_behavior, time, dt);
  }
}

}  // namespace Strategy
}  // namespace rhoban_ssl
