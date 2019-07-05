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

#pragma once

#include <manager/manager_with_game_state.h>

namespace rhoban_ssl
{
namespace manager
{
class LordOfDarkness : public ManagerWithGameState
{
private:
  std::vector<std::list<std::string> > offensive_strats_;

  // defensive strats:
  std::vector<std::list<std::string> > defensive_strats_;

  // halt_strat
  std::vector<std::list<std::string> > halt_strats_;

  std::vector<std::list<std::string> > kickoff_ally_strats_;

  std::vector<std::list<std::string> > kickoff_opponent_strats_;

  std::list<std::string> future_strats_;

  // penalty ally
  std::vector<std::list<std::string> > penalty_strats_a_;
  // penalty opponent
  std::vector<std::list<std::string> > penalty_strats_o_;
  // goale
  std::vector<std::list<std::string> > goalie_strats_;
  // stop
  std::vector<std::list<std::string> > stop_strats_;
  // kick
  std::vector<std::list<std::string> > kick_strats_;
  // kick_strats_indirect
  std::vector<std::list<std::string> > kick_strats_indirect_;

  std::vector<std::list<std::string> > direct_opponent_strats_;

  std::string PROTECT_BALL = "gobelin";

  bool ball_was_in_ally_part_ = true;

public:
  LordOfDarkness(std::string name);

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

  virtual void startPreparePenaltyAlly();
  virtual void startPreparePenaltyOpponent();

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

  virtual void continuePreparePenaltyAlly();
  virtual void continuePreparePenaltyOpponent();

  virtual void continuePenaltyAlly();
  virtual void continuePenaltyOpponent();

  virtual ~LordOfDarkness();
};

};  // namespace manager
};  // namespace rhoban_ssl
