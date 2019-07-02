/*
    This file is part of SSL.

    Copyright 2019 SCHMITZ Etienne (hello@etienne-schmitz.com)

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

namespace rhoban_ssl
{
namespace manager
{
Match::Match(std::string name) : ManagerWithGameState(name)
{
  // Register strategy.
  registerStrategy(strategy::Halt::name, std::shared_ptr<strategy::Strategy>(new strategy::Halt()));
}

void Match::startStop()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = stop_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void Match::startRunning()
{
  setBallAvoidanceForAllRobots(false);
  // if (ballPosition().getX() <= 0)
  //  {
  //    //+ 1 because the method getValidPlayerIds() doesn't count goalie.
  //    future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  //    ball_was_in_ally_part_ = true;
  //  }
  //  else
  //  {
  //    future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
  //    ball_was_in_ally_part_ = false;
  //  }
  declareAndAssignNextStrategies(future_strats_);
}
void Match::startHalt()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = halt_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void Match::startDirectKickAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = kick_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void Match::startDirectKickOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void Match::startIndirectKickAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = kick_strats_indirect_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void Match::startIndirectKickOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void Match::startPrepareKickoffAlly()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void Match::startPrepareKickoffOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void Match::startKickoffAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void Match::startKickoffOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void Match::startPenaltyAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = penalty_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void Match::startPenaltyOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = stop_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

// Continue

void Match::continueStop()
{
}

void Match::continueRunning()
{
  //  if (ballPosition().getX() <= 0 and not(ball_was_in_ally_part_))
  //  {
  //    clearStrategyAssignement();
  //    future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  //    ball_was_in_ally_part_ = true;
  //   declareAndAssignNextStrategies(future_strats_);
  //  }
  //  else if (ballPosition().getX() > 0 and ball_was_in_ally_part_)
  //  {
  //   clearStrategyAssignement();
  //    future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
  //    ball_was_in_ally_part_ = false;
  //    declareAndAssignNextStrategies(future_strats_);
  //  }
}
void Match::continueHalt()
{
}

void Match::continueDirectKickAlly()
{
}
void Match::continueDirectKickOpponent()
{
}

void Match::continueIndirectKickAlly()
{
}
void Match::continueIndirectKickOpponent()
{
}

void Match::continuePrepareKickoffAlly()
{
}
void Match::continuePrepareKickoffOpponent()
{
}

void Match::continueKickoffAlly()
{
}
void Match::continueKickoffOpponent()
{
}

void Match::continuePenaltyAlly()
{
}
void Match::continuePenaltyOpponent()
{
}

Match::~Match()
{
}

};  // namespace manager
};  // namespace rhoban_ssl
