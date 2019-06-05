/*
This file is part of SSL.

Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#pragma once

#include <manager/manager_with_game_state.h>

namespace rhoban_ssl
{
namespace manager
{
class PlanVeschambres : public ManagerWithGameState
{
private:

  // penalty
  std::vector<std::list<std::string> > penalty_strats_;
  // goale
  std::vector<std::list<std::string> > goalie_strats_;
  // offensive
  std::vector<std::list<std::string> > offensive_strats_;
  // stop
  std::vector<std::list<std::string> > stop_strats_;
  // halt
  std::vector<std::list<std::string> > halt_strats_;
  // defensive
  std::vector<std::list<std::string> > defensive_strats_;
  // kick
  std::vector<std::list<std::string> > kick_strats_;
  // kick_strats_indirect
  std::vector<std::list<std::string> > kick_strats_indirect_;

  bool ball_was_in_ally_part_ = true;

  std::list<std::string> future_strats_;

public:
  PlanVeschambres(std::string name);

  // Begin of a new state
  virtual void startStop();
  virtual void startRunning();
  virtual void startHalt();

  virtual void startDirectKickAlly();
  virtual void startDirectKickOpponent();

  virtual void startIndirectKickAlly();
  virtual void startIndirectKickOpponent();

  virtual void startPrepareKickoffAlly();
  virtual void startPrepareKickoffOpponent();

  virtual void startKickoffAlly();
  virtual void startKickoffOpponent();

  virtual void startPenaltyAlly();
  virtual void startPenaltyOpponent();

  // During a state
  virtual void continueStop();
  virtual void continueRunning();
  virtual void continueHalt();

  virtual void continueDirectKickAlly();
  virtual void continueDirectKickOpponent();

  virtual void continueIndirectKickAlly();
  virtual void continueIndirectKickOpponent();

  virtual void continuePrepareKickoffAlly();
  virtual void continuePrepareKickoffOpponent();

  virtual void continueKickoffAlly();
  virtual void continueKickoffOpponent();

  virtual void continuePenaltyAlly();
  virtual void continuePenaltyOpponent();

  virtual ~PlanVeschambres();
};

};  // namespace manager
};  // namespace rhoban_ssl
