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
namespace Strategy
{
Union::Union(Ai::AiData& ai_data) : Strategy(ai_data), min(0), max(0)
{
}

void Union::add_goalie_strategy(std::shared_ptr<Strategy> strategy)
{
  min += strategy->min_robots();
  strategy_with_goal = strategy;
}

void Union::add_strategy(std::shared_ptr<Strategy> strategy)
{
  if (strategy->max_robots() < 0)
  {
    max = -1;
  }
  else
  {
    if (max >= 0)
    {
      max += strategy->max_robots();
    }
  }
  strategies_without_goal.push_back(strategy);
}

int Union::min_robots() const
{
  return min;
}
int Union::max_robots() const
{
  return max;
}
Goalie_need Union::needs_goalie() const
{
  if (strategy_with_goal)
  {
    return Goalie_need::YES;
  }
  else
  {
    return Goalie_need::NO;
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

void Union::assign_behavior_to_robots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (strategy_with_goal)
  {
    strategy_with_goal->assign_behavior_to_robots(assign_behavior, time, dt);
  }
  for (std::shared_ptr<Strategy>& elem : strategies_without_goal)
  {
    elem->assign_behavior_to_robots(assign_behavior, time, dt);
  }
}

}  // namespace Strategy
}  // namespace rhoban_ssl
