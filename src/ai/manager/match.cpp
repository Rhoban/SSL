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

#include "match.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/placer.h>
#include <strategy/prepare_kickoff.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/defensor.h>
#include <robot_behavior/striker.h>
#include <core/collection.h>
#include <core/print_collection.h>

#define GOALIE "Goalie"
#define DEFENSOR1 "Defensor1"
#define DEFENSOR2 "Defensor2"
#define STRIKER "Striker"

namespace rhoban_ssl
{
namespace manager
{
Match::Match(ai::AiData& ai_data, const GameState& game_state)
  : Manager(ai_data), game_state_(game_state), last_game_state_changement_(0)
{
  registerStrategy(strategy::Halt::name, std::shared_ptr<strategy::Strategy>(new strategy::Halt(ai_data)));
  registerStrategy(strategy::Tare_and_synchronize::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::Tare_and_synchronize(ai_data)));
  registerStrategy(strategy::Prepare_kickoff::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::Prepare_kickoff(ai_data)));
  registerStrategy(GOALIE, std::shared_ptr<strategy::Strategy>(new strategy::From_robot_behavior(
                                ai_data,
                                [&](double time, double dt) {
                                  Robot_behavior::Goalie* goalie = new Robot_behavior::Goalie(ai_data);
                                  return std::shared_ptr<Robot_behavior::RobotBehavior>(goalie);
                                },
                                true  // it is a goal
                                )));
  registerStrategy(DEFENSOR1, std::shared_ptr<strategy::Strategy>(new strategy::From_robot_behavior(
                                   ai_data,
                                   [&](double time, double dt) {
                                     Robot_behavior::Defensor* defensor = new Robot_behavior::Defensor(ai_data);
                                     return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
                                   },
                                   false  // it is not a goal
                                   )));
  registerStrategy(DEFENSOR2, std::shared_ptr<strategy::Strategy>(new strategy::From_robot_behavior(
                                   ai_data,
                                   [&](double time, double dt) {
                                     Robot_behavior::Defensor* defensor = new Robot_behavior::Defensor(ai_data);
                                     return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
                                   },
                                   false  // it is not a goal
                                   )));
  registerStrategy(STRIKER, std::shared_ptr<strategy::Strategy>(new strategy::From_robot_behavior(
                                 ai_data,
                                 [&](double time, double dt) {
                                   Robot_behavior::Striker* striker = new Robot_behavior::Striker(ai_data);
                                   return std::shared_ptr<Robot_behavior::RobotBehavior>(striker);
                                 },
                                 false  // it is not a goal
                                 )));
  assignStrategy(strategy::Halt::name, 0.0,
                  getTeamIds());  // TODO TIME !
}

void Match::chooseAStrategy(double time)
{
  if (game_state_.edge_entropy() > last_game_state_changement_)
  {
    clearStrategyAssignement();
    if (game_state_.getState() == state_name::halt)
    {
      assignStrategy(strategy::Halt::name, time, get_valid_team_ids());
    }
    else if (game_state_.getState() == state_name::stop)
    {
      if (getValidTeamIds().size() > 0)
      {
        if (not(getStrategy<strategy::Tare_and_synchronize>().is_tared_and_synchronized()))
        {
          assignStrategy(strategy::Tare_and_synchronize::name, time, getValidPlayerIds());
        }
        else
        {
          placeAllTheRobots(time, future_strats_);
        }
      }
    }
    else if (game_state_.getState() == state_name::prepare_kickoff)
    {
      if (getTeam() == game_state_.kickoffTeam())
      {
        getStrategy<strategy::Prepare_kickoff>().set_kicking(true);
      }
      else
      {
        getStrategy<strategy::Prepare_kickoff>().set_kicking(false);
      }
      future_strats_ = { strategy::Prepare_kickoff::name };
      declareAndAssignNextStrategies(future_strats_);
    }
    else if (game_state_.getState() == state_name::penalty)
    {
    }
    else if (game_state_.getState() == state_name::running)
    {
      future_strats_ = { GOALIE, DEFENSOR1, STRIKER };
      declareAndAssignNextStrategies(future_strats_);
    }
    else if (game_state_.getState() == state_name::STATE_TIMEOUT)
    {
      assignStrategy(strategy::Halt::name, time, getValidTeamIds());
    }
    last_game_state_changement_ = game_state_.edge_entropy();
  }
}

void Match::update(double time)
{
  // update_strategies(time);
  updateCurrentStrategies(time);
  chooseAStrategy(time);
}

Match::~Match()
{
}

};  // namespace Manager
};  // namespace RhobanSSL
