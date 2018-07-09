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

#ifndef __MANAGER__SHINTAEYONG__H__
#define __MANAGER__SHINTAEYONG__H__

#include <manager/Manager.h>
#include <referee/Referee.h>
#include <core/machine_state.h>

namespace RhobanSSL {
namespace Manager {

  class ShinTaeYong : public Manager {
  private:
    const Referee & referee;

    std::string strategy_applied = "";

    rhoban_geometry::Point ball_last_position;

    unsigned int last_referee_changement;

    std::list<std::string> future_strats;

    struct state_name {
      static const constexpr char* stop = "stop" ;
      static const constexpr char* normal_start = "normal_start" ;
      static const constexpr char* halt = "halt" ;
      static const constexpr char* free_kick_ally = "free_kick_ally" ;
      static const constexpr char* free_kick_opponent = "free_kick_opponent" ;
      static const constexpr char* kickoff_ally = "kickoff_ally" ;
      static const constexpr char* kickoff_opponent = "kickoff_opponent" ;
    };

    struct edge_name {
      static const constexpr char* force_start = "force_start" ;

      static const constexpr char* normal_to_stop = "normal_to_stop" ;
      static const constexpr char* normal_to_halt = "normal_to_halt" ;

      static const constexpr char* ball_move_ally = "ball_move_ally" ;
      static const constexpr char* ball_move_opponent = "ball_move_opponent" ;

      static const constexpr char* free_kick_ally_to_halt = "free_kick_ally_to_halt" ;
      static const constexpr char* free_kick_opponent_to_halt = "free_kick_opponent_to_halt" ;

      static const constexpr char* free_kick_ally_to_stop = "free_kick_ally_to_stop" ;
      static const constexpr char* free_kick_opponent_to_stop = "free_kick_opponent_to_stop" ;

      static const constexpr char* halt_to_stop = "halt_to_stop" ;

      static const constexpr char* kickoff_ally_to_normal = "kickoff_ally_to_normal" ;
      static const constexpr char* stop_to_kickoff_ally = "stop_to_kickoff_ally" ;

      static const constexpr char* direct_ally = "direct_ally" ;
      static const constexpr char* indirect_ally = "indirect_ally" ;
      static const constexpr char* penalty_ally = "penalty_ally" ;

      static const constexpr char* kickoff_opponent_to_normal = "kickoff_opponent_to_normal" ;
      static const constexpr char* stop_to_kickoff_opponent = "stop_to_kickoff_opponent" ;

      static const constexpr char* direct_opponent = "direct_opponent" ;
      static const constexpr char* indirect_opponent = "indirect_opponent" ;
      static const constexpr char* penalty_opponent = "penalty_opponent" ;
    };

    typedef construct_machine_state_infrastructure<
    std::string, Ai::AiData, Ai::AiData
    > machine_state_infrastructure;

    machine_state_infrastructure::MachineState machine;


  public:

    ShinTaeYong(
      Ai::AiData & ai_data,
      const Referee & referee
    );

    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);

    virtual ~ShinTaeYong();

  };

};
};

#endif
