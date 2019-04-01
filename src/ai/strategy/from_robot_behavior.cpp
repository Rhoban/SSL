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
namespace strategy
{
const std::string FromRobotBehavior::name = "From_robot_behavior";

FromRobotBehavior::FromRobotBehavior(
    ai::AiData& ai_data,
    std::function<std::shared_ptr<robot_behavior::RobotBehavior>(double time, double dt)> robot_behavior_allocator,
    bool is_goalie)
  : Strategy(ai_data), robot_behavior_allocator_(robot_behavior_allocator), is_goalie_(is_goalie){};

FromRobotBehavior::FromRobotBehavior(
    ai::AiData& ai_data,
    std::function<std::shared_ptr<robot_behavior::RobotBehavior>(double time, double dt)> robot_behavior_allocator,
    const rhoban_geometry::Point& starting_linear_position, const ContinuousAngle& starting_angular_position,
    bool is_goalie)
  : FromRobotBehavior(ai_data, robot_behavior_allocator, is_goalie)
{
  setStartingPosition(starting_linear_position, starting_angular_position);
};

GoalieNeed FromRobotBehavior::needsGoalie() const
{
  return is_goalie_ ? GoalieNeed::YES : GoalieNeed::NO;
}

int FromRobotBehavior::minRobots() const
{
  return is_goalie_ ? 0 : 1;
}

int FromRobotBehavior::maxRobots() const
{
  return is_goalie_ ? 0 : 1;
}

void FromRobotBehavior::start(double time)
{
  DEBUG("START STRATEGY FROM BEHAVIOR "
        "TODO");
  behavior_has_been_assigned_ = false;
}

void FromRobotBehavior::stop(double time)
{
  DEBUG("STOP STRATEGY FROM BEHAVIOR "
        "TODO");
}

void FromRobotBehavior::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (!behavior_has_been_assigned_)
  {
    if (is_goalie_)
    {
      DEBUG("GOALIE : " << getGoalie());
      if (have_to_manage_the_goalie())
      {
        assign_behavior(getGoalie(), robot_behavior_allocator_(time, dt));
      }
    }
    else
    {
      assign_behavior(playerId(0), robot_behavior_allocator_(time, dt));
    }
    behavior_has_been_assigned_ = true;
  }
}

std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
FromRobotBehavior::getStartingPositions(int number_of_avalaible_robots) const
{
  std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> > result;
  if (starting_position_.is_defined)
  {
    result.push_back(std::pair<rhoban_geometry::Point, ContinuousAngle>(starting_position_.linear_position,
                                                                        starting_position_.angular_position));
  }
  return result;
}

bool FromRobotBehavior::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                                           ContinuousAngle& angular_position) const
{
  assert(is_goalie_);  // This function should not called since strategy doesn't devaler a goalie.
  if (starting_position_.is_defined)
  {
    linear_position = starting_position_.linear_position;
    angular_position = starting_position_.angular_position;
  }
  return starting_position_.is_defined;
}

void FromRobotBehavior::setStartingPosition(const rhoban_geometry::Point& linear_position,
                                                const ContinuousAngle& angular_position)
{
  starting_position_.is_defined = true;
  starting_position_.linear_position = linear_position;
  starting_position_.angular_position = angular_position;
}

FromRobotBehavior::~FromRobotBehavior()
{
}

};  // namespace Strategy
};  // namespace rhoban_ssl
