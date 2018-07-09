

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

#include "shin_tae_yong.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/placer.h>
#include <strategy/prepare_kickoff.h>
#include <strategy/from_robot_behavior.h>

#include <strategy/offensive.h>
#include <strategy/defensive.h>
#include <strategy/defensive_2.h>
#include <strategy/mur_stop.h>
#include <strategy/mur.h>
#include <strategy/mur_2.h>
#include <strategy/mur_2_passif.h>
#include <strategy/attaque_with_support.h>
#include <strategy/attaque_with_support_ms.h>
#include <strategy/striker_with_support.h>
#include <strategy/striker_v2.h>
#include <strategy/striker_kick.h>

#include <strategy/goalie_strat.h>

#include <robot_behavior/goalie.h>
#include <robot_behavior/protect_ball.h>

#include <core/collection.h>
#include <core/print_collection.h>

#define GOALIE "goalie"
#define PROTECT_BALL "protect_ball"

namespace RhobanSSL
{
namespace Manager
{

ShinTaeYong::ShinTaeYong(
    Ai::AiData &ai_data,
    const Referee &referee) : Manager(ai_data),
                              referee(referee),
                              last_referee_changement(0),
                              machine(ai_data, ai_data)
{
  //STATES
  machine.add_state(
    state_name::stop,
    [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      DEBUG(state_name::stop);
    }
  );
  machine.add_state(
    state_name::normal_start,
    [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      DEBUG(state_name::normal_start);
    }
  );
  machine.add_state(
    state_name::halt,
    [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      DEBUG(state_name::halt);
    }
  );
  machine.add_state(
    state_name::free_kick_ally,
    [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      DEBUG(state_name::free_kick_ally);
    }
  );
  machine.add_state(
    state_name::free_kick_opponent,
    [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      DEBUG(state_name::free_kick_opponent);
    }
  );
  machine.add_state(
    state_name::kickoff_ally,
    [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      DEBUG(state_name::kickoff_ally);
    }
  );
  machine.add_state(
    state_name::kickoff_opponent,
    [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      DEBUG(state_name::kickoff_opponent);
    }
  );

  //EDGES
  machine.add_edge(
    edge_name::force_start,
    state_name::stop, state_name::normal_start,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      //TODO; referee_iD::STATE_FORCE_START
      return false;
    }
  );

  machine.add_edge(
    edge_name::normal_to_stop,
    state_name::normal_start, state_name::stop,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::normal_to_halt,
    state_name::normal_start, state_name::halt,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );

  machine.add_edge(
    edge_name::ball_move_ally,
    state_name::free_kick_ally, state_name::normal_start,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::ball_move_opponent,
    state_name::free_kick_opponent, state_name::normal_start,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );

  machine.add_edge(
    edge_name::free_kick_ally_to_halt,
    state_name::free_kick_ally, state_name::halt,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::free_kick_opponent_to_halt,
    state_name::free_kick_opponent, state_name::halt,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );

  machine.add_edge(
    edge_name::free_kick_ally_to_stop,
    state_name::free_kick_ally, state_name::stop,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::free_kick_opponent_to_stop,
    state_name::free_kick_opponent, state_name::stop,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );

  machine.add_edge(
    edge_name::halt_to_stop,
    state_name::halt, state_name::stop,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );

  machine.add_edge(
    edge_name::kickoff_ally_to_normal,
    state_name::kickoff_ally, state_name::normal_start,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::stop_to_kickoff_ally,
    state_name::stop, state_name::kickoff_ally,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );

  machine.add_edge(
    edge_name::direct_ally,
    state_name::stop, state_name::free_kick_ally,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::indirect_ally,
    state_name::stop, state_name::free_kick_ally,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::penalty_ally,
    state_name::stop, state_name::free_kick_ally,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );

  machine.add_edge(
    edge_name::kickoff_opponent_to_normal,
    state_name::kickoff_opponent, state_name::normal_start,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::stop_to_kickoff_opponent,
    state_name::stop, state_name::kickoff_opponent,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );


  machine.add_edge(
    edge_name::direct_opponent,
    state_name::stop, state_name::free_kick_opponent,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::indirect_opponent,
    state_name::stop, state_name::free_kick_opponent,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );
  machine.add_edge(
    edge_name::penalty_opponent,
    state_name::stop, state_name::free_kick_opponent,
    [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
      return false;
    }
  );

  machine.export_to_file("/tmp/shin_tae_yong.dot");
}

void ShinTaeYong::analyse_data(double time)
{
    // We change the point of view of the team
    change_team_and_point_of_view(
        referee.get_team_color(get_team_name()),
        referee.blue_have_it_s_goal_on_positive_x_axis());
    change_ally_and_opponent_goalie_id(
        referee.blue_goalie_id(),
        referee.yellow_goalie_id());
}

void ShinTaeYong::choose_a_strategy(double time)
{

}

void ShinTaeYong::update(double time)
{
    //update_strategies(time);
    update_current_strategies(time);
    analyse_data(time);
    choose_a_strategy(time);
}

ShinTaeYong::~ShinTaeYong() {}

}; // namespace Manager
}; // namespace RhobanSSL
