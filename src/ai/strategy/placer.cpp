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

#include "placer.h"
#include <robot_behavior/factory.h>
#include <core/print_collection.h>
#include <robot_behavior/consign_follower.h>

namespace rhoban_ssl
{
namespace strategy
{
Placer::Placer(ai::AiData& ai_data) : Strategy(ai_data), goalie_is_defined_(false)
{
}

const std::string Placer::name = "placer";

int Placer::minRobots() const
{
  return 0;
}
int Placer::maxRobots() const
{
  return -1;
}
GoalieNeed Placer::needsGoalie() const
{
  return GoalieNeed::IF_POSSIBLE;
}

void Placer::start(double time)
{
  DEBUG("START PREPARE PLACER");
  behavior_has_been_assigned = false;
}

void Placer::stop(double time)
{
  DEBUG("STOP PREPARE PLACER");
}

void Placer::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (!behavior_has_been_assigned)
  {
    if (have_to_manage_the_goalie())
    {
      robot_behavior::ConsignFollower* follower =
          robot_behavior::Factory::fixedConsignFollower(ai_data_, goalie_linear_position_, goalie_angular_position_);
      follower->avoidTheBall(true);
      assign_behavior(getGoalie(), std::shared_ptr<robot_behavior::RobotBehavior>(follower));
    }

    int nb_players = getPlayerIds().size();
    for (int i = 0; i < nb_players; i++)
    {
      int id = playerId(i);
      robot_behavior::ConsignFollower* follower = robot_behavior::Factory::fixedConsignFollower(
          ai_data_, player_positions_[id].first, player_positions_[id].second);
      follower->avoidTheBall(true);
      assign_behavior(id, std::shared_ptr<robot_behavior::RobotBehavior>(follower));
    }
    behavior_has_been_assigned = true;
  }
}

Placer::~Placer()
{
}

/*
 *
 *
 */
void Placer::setPositions(const std::vector<int>& robot_affectations,
                          const std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> >& robot_consigns)
{
  assert(robot_affectations.size() == robot_consigns.size());

  player_positions_.clear();
  for (unsigned int i = 0; i < robot_affectations.size(); i++)
  {
    player_positions_[robot_affectations[i]] = robot_consigns[i];
  }
}

void Placer::setGoaliePositions(const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position)
{
  this->goalie_linear_position_ = linear_position;
  this->goalie_angular_position_ = angular_position;
}

std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Placer::getStartingPositions(int number_of_avalaible_robots) const
{
  std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> > result;
  int cpt = 0;
  for (const std::pair<rhoban_geometry::Point, ContinuousAngle>& elem : starting_positions_)
  {
    if (cpt >= number_of_avalaible_robots)
    {
      break;
    }
    result.push_back(elem);
    cpt++;
  }
  return result;
}

void Placer::setStartingPositions(
    const std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >& starting_positions)
{
  this->starting_positions_ = starting_positions;
}

bool Placer::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                          ContinuousAngle& angular_position) const
{
  if (goalie_is_defined_)
  {
    linear_position = starting_position_for_goalie_.first;
    angular_position = starting_position_for_goalie_.second;
  }
  return goalie_is_defined_;
}

void Placer::setStartingPositionForGoalie(const rhoban_geometry::Point& linear_position,
                                          const ContinuousAngle& angular_position)
{
  goalie_is_defined_ = true;
  starting_position_for_goalie_.first = linear_position;
  starting_position_for_goalie_.second = angular_position;
}

}  // namespace strategy
}  // namespace rhoban_ssl
