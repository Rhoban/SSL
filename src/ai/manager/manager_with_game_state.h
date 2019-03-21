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

#ifndef __MANAGER__WITH_GAME_STATE__H__
#define __MANAGER__WITH_GAME_STATE__H__

#include "Manager.h"
#include "referee/game_state.h"

namespace RhobanSSL
{
namespace Manager
{
class ManagerWithGameState : public Manager
{
private:
  const GameState& game_state;

  unsigned int last_change_stamp;

public:
  ManagerWithGameState(Ai::AiData& ai_data, const GameState& game_state);

  void update(double time);
  void analyse_data(double time);
  void choose_a_strategy(double time);

  // Begin of a new state
  virtual void start_stop() = 0;
  virtual void start_running() = 0;
  virtual void start_halt() = 0;

  virtual void start_direct_kick_ally() = 0;
  virtual void start_direct_kick_opponent() = 0;

  virtual void start_indirect_kick_ally() = 0;
  virtual void start_indirect_kick_opponent() = 0;

  virtual void start_prepare_kickoff_ally() = 0;
  virtual void start_prepare_kickoff_opponent() = 0;

  virtual void start_kickoff_ally() = 0;
  virtual void start_kickoff_opponent() = 0;

  virtual void start_penalty_ally() = 0;
  virtual void start_penalty_opponent() = 0;

  // During a state
  virtual void continue_stop() = 0;
  virtual void continue_running() = 0;
  virtual void continue_halt() = 0;

  virtual void continue_direct_kick_ally() = 0;
  virtual void continue_direct_kick_opponent() = 0;

  virtual void continue_indirect_kick_ally() = 0;
  virtual void continue_indirect_kick_opponent() = 0;

  virtual void continue_prepare_kickoff_ally() = 0;
  virtual void continue_prepare_kickoff_opponent() = 0;

  virtual void continue_kickoff_ally() = 0;
  virtual void continue_kickoff_opponent() = 0;

  virtual void continue_penalty_ally() = 0;
  virtual void continue_penalty_opponent() = 0;

  virtual ~ManagerWithGameState();
};

};  // namespace Manager
};  // namespace RhobanSSL

#endif
