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
namespace Manager
{
class PlanVeschambres : public ManagerWithGameState
{
private:
  const GameState& game_state;

  // penalty
  std::vector<std::list<std::string> > penalty_strats;
  // goale
  std::vector<std::list<std::string> > goalie_strats;
  // offensive
  std::vector<std::list<std::string> > offensive_strats;
  // stop
  std::vector<std::list<std::string> > stop_strats;
  // halt
  std::vector<std::list<std::string> > halt_strats;
  // defensive
  std::vector<std::list<std::string> > defensive_strats;
  // kick
  std::vector<std::list<std::string> > kick_strats;
  // kick_strats_indirect
  std::vector<std::list<std::string> > kick_strats_indirect;

  bool ball_was_in_ally_part = true;

  std::list<std::string> future_strats;

public:
  PlanVeschambres(ai::AiData& ai_data, const GameState& game_state);

  // Begin of a new state
  virtual void start_stop();
  virtual void start_running();
  virtual void start_halt();

  virtual void start_direct_kick_ally();
  virtual void start_direct_kick_opponent();

  virtual void start_indirect_kick_ally();
  virtual void start_indirect_kick_opponent();

  virtual void start_prepare_kickoff_ally();
  virtual void start_prepare_kickoff_opponent();

  virtual void start_kickoff_ally();
  virtual void start_kickoff_opponent();

  virtual void start_penalty_ally();
  virtual void start_penalty_opponent();

  // During a state
  virtual void continue_stop();
  virtual void continue_running();
  virtual void continue_halt();

  virtual void continue_direct_kick_ally();
  virtual void continue_direct_kick_opponent();

  virtual void continue_indirect_kick_ally();
  virtual void continue_indirect_kick_opponent();

  virtual void continue_prepare_kickoff_ally();
  virtual void continue_prepare_kickoff_opponent();

  virtual void continue_kickoff_ally();
  virtual void continue_kickoff_opponent();

  virtual void continue_penalty_ally();
  virtual void continue_penalty_opponent();

  virtual ~PlanVeschambres();
};

};  // namespace Manager
};  // namespace rhoban_ssl
