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

#include "from_robot_behavior.h"

namespace rhoban_ssl
{
namespace Strategy
{
const std::string From_robot_behavior::name = "From_robot_behavior";

From_robot_behavior::From_robot_behavior(
    ai::AiData& ai_data,
    std::function<std::shared_ptr<Robot_behavior::RobotBehavior>(double time, double dt)> robot_behavior_allocator,
    bool is_goalie)
  : Strategy(ai_data), robot_behavior_allocator(robot_behavior_allocator), is_goalie(is_goalie){};

From_robot_behavior::From_robot_behavior(
    ai::AiData& ai_data,
    std::function<std::shared_ptr<Robot_behavior::RobotBehavior>(double time, double dt)> robot_behavior_allocator,
    const rhoban_geometry::Point& starting_linear_position, const ContinuousAngle& starting_angular_position,
    bool is_goalie)
  : From_robot_behavior(ai_data, robot_behavior_allocator, is_goalie)
{
  set_starting_position(starting_linear_position, starting_angular_position);
};

Goalie_need From_robot_behavior::needs_goalie() const
{
  return is_goalie ? Goalie_need::YES : Goalie_need::NO;
}

int From_robot_behavior::min_robots() const
{
  return is_goalie ? 0 : 1;
}

int From_robot_behavior::max_robots() const
{
  return is_goalie ? 0 : 1;
}

void From_robot_behavior::start(double time)
{
  DEBUG("START STRATEGY FROM BEHAVIOR "
        "TODO");
  behavior_has_been_assigned = false;
}

void From_robot_behavior::stop(double time)
{
  DEBUG("STOP STRATEGY FROM BEHAVIOR "
        "TODO");
}

void From_robot_behavior::assign_behavior_to_robots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (!behavior_has_been_assigned)
  {
    if (is_goalie)
    {
      DEBUG("GOALIE : " << get_goalie());
      if (have_to_manage_the_goalie())
      {
        assign_behavior(get_goalie(), robot_behavior_allocator(time, dt));
      }
    }
    else
    {
      assign_behavior(player_id(0), robot_behavior_allocator(time, dt));
    }
    behavior_has_been_assigned = true;
  }
}

std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
From_robot_behavior::get_starting_positions(int number_of_avalaible_robots) const
{
  std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> > result;
  if (starting_position.is_defined)
  {
    result.push_back(std::pair<rhoban_geometry::Point, ContinuousAngle>(starting_position.linear_position,
                                                                        starting_position.angular_position));
  }
  return result;
}

bool From_robot_behavior::get_starting_position_for_goalie(rhoban_geometry::Point& linear_position,
                                                           ContinuousAngle& angular_position) const
{
  assert(is_goalie);  // This function should not called since strategy doesn't devaler a goalie.
  if (starting_position.is_defined)
  {
    linear_position = starting_position.linear_position;
    angular_position = starting_position.angular_position;
  }
  return starting_position.is_defined;
}

void From_robot_behavior::set_starting_position(const rhoban_geometry::Point& linear_position,
                                                const ContinuousAngle& angular_position)
{
  starting_position.is_defined = true;
  starting_position.linear_position = linear_position;
  starting_position.angular_position = angular_position;
}

From_robot_behavior::~From_robot_behavior()
{
}

};  // namespace Strategy
};  // namespace rhoban_ssl
