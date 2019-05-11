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

#include "manager.h"
#include "referee/game_state.h"

namespace rhoban_ssl
{
namespace manager
{
class ManagerWithGameState : public Manager
{
private:
  const GameState& game_state_;

  unsigned int last_change_stamp_;

public:
  ManagerWithGameState(ai::AiData& ai_data, const GameState& game_state);

  void update(double time);
  void analyseData(double time);
  void chooseAStrategy(double time);

  // Begin of a new state
  virtual void startStop() = 0;
  virtual void startRunning() = 0;
  virtual void startHalt() = 0;

  virtual void startDirectKickAlly() = 0;
  virtual void startDirectKickOpponent() = 0;

  virtual void startIndirectKickAlly() = 0;
  virtual void startIndirectKickOpponent() = 0;

  virtual void startPrepareKickoffAlly() = 0;
  virtual void startPrepareKickoffOpponent() = 0;

  virtual void startKickoffAlly() = 0;
  virtual void startKickoffOpponent() = 0;

  virtual void startPenaltyAlly() = 0;
  virtual void startPenaltyOpponent() = 0;

  // During a state
  virtual void continueStop() = 0;
  virtual void continueRunning() = 0;
  virtual void continueHalt() = 0;

  virtual void continueDirectKickAlly() = 0;
  virtual void continueDirectKickOpponent() = 0;

  virtual void continueIndirectKickAlly() = 0;
  virtual void continueIndirectKickOpponent() = 0;

  virtual void continuePrepareKickoffAlly() = 0;
  virtual void continuePrepareKickoffOpponent() = 0;

  virtual void continueKickoffAlly() = 0;
  virtual void continueKickoffOpponent() = 0;

  virtual void continuePenaltyAlly() = 0;
  virtual void continuePenaltyOpponent() = 0;

  virtual ~ManagerWithGameState();
};

};  // namespace manager
};  // namespace rhoban_ssl
