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
class DumbManager : public ManagerWithGameState
{
private:
  // dumb_strat
  std::vector<std::list<std::string> > dumb_strats_;

  //hal_strat
  std::vector<std::list<std::string> > halt_strats_;

  std::list<std::string> future_strats_;

public:
  DumbManager(std::string name);

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

  virtual ~DumbManager();
};

};  // namespace manager
};  // namespace rhoban_ssl
