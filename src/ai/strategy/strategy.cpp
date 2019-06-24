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

#include "strategy.h"

namespace rhoban_ssl
{
namespace strategy
{
Strategy::Strategy()
  : GameInformations(), goalie_id_(ai::Config::default_goalie_id), manage_a_goalie_(false), goalie_opponent_id_(-1)
{
}

void Strategy::setGoalie(uint id, bool to_be_managed)
{
  goalie_id_ = id;
  manage_a_goalie_ = to_be_managed;
}

bool Strategy::haveToManageTheGoalie() const
{
  return ((needsGoalie() != GoalieNeed::NO) and manage_a_goalie_);
}

void Strategy::setGoalieOpponent(uint id)
{
  goalie_opponent_id_ = int(id);
}

uint Strategy::getGoalie() const
{
  return goalie_id_;
}

int Strategy::getGoalieOpponent() const
{
  return goalie_opponent_id_;
}

void Strategy::setRobotAffectation(const std::vector<int>& robot_ids)
{
  this->player_ids_ = robot_ids;
}
const std::vector<int>& Strategy::getPlayerIds() const
{
  return player_ids_;
}

int Strategy::robotId(int id) const
{
  assert(0 <= id and static_cast<unsigned int>(id) < player_ids_.size());  // Whent that line make an assertion, that
                                                                           // means, you don't have updated
  // the minRobots() implementation inside your strategy code.
  return player_ids_[id];
}

int Strategy::playerId(int id) const
{
  assert(0 <= id and static_cast<unsigned int>(id) < player_ids_.size());  // Whent that line make an assertion, that
                                                                           // means, you don't have updated
  // the minRobots() implementation inside your strategy code.
  return player_ids_[id];
}

std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Strategy::getStartingPositions(int number_of_avalaible_robots) const
{
  return std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >();
};

bool Strategy::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                            ContinuousAngle& angular_position) const
{
  return false;
}

Json::Value Strategy::getParameters()
{
}

void Strategy::setParameters(Json::Value)
{
}

Strategy::~Strategy()
{
}

rhoban_ssl::annotations::Annotations Strategy::getAnnotations() const
{
  return rhoban_ssl::annotations::Annotations();
}

}  // namespace strategy
}  // namespace rhoban_ssl
