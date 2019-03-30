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

#include "game_state.h"
#include <debug.h>
#include <core/print_collection.h>
#include "print_protobuf_referee.h"
#include <game_informations.h>

namespace rhoban_ssl
{
GameStateData::GameStateData() : datas(2), last_time(0.0), last_command_counter(0)
{
}

const SSL_Referee& GameStateData::current() const
{
  return datas[0];
}

const SSL_Referee& GameStateData::old() const
{
  return datas[1];
}

bool GameStateData::command_is_new() const
{
  return (last_time < current().packet_timestamp()) and (last_command_counter < current().command_counter());
}

template <SSL_Referee_Command E>
bool command_is_(const GameStateData& game_state_data, unsigned int run_number, unsigned int atomic_run_number)
{
  return (game_state_data.command_is_new()) and (game_state_data.current().command() == E);
}

template <SSL_Referee_Command E1, SSL_Referee_Command E2>
bool command_is_one_of_(const GameStateData& game_state_data, unsigned int run_number, unsigned int atomic_run_number)
{
  return (game_state_data.command_is_new()) and
         ((game_state_data.current().command() == E1) or (game_state_data.current().command() == E2));
}

template <SSL_Referee_Command E1, SSL_Referee_Command E2, SSL_Referee_Command E3, SSL_Referee_Command E4>
bool command_is_one_of_4_(const GameStateData& game_state_data, unsigned int run_number, unsigned int atomic_run_number)
{
  return (game_state_data.command_is_new()) and
         ((game_state_data.current().command() == E1) or (game_state_data.current().command() == E2) or
          (game_state_data.current().command() == E3) or (game_state_data.current().command() == E4));
}

GameState::GameState(Ai::AiData& ai_data)
  : ai_data(ai_data)
  , blueTeamOnPositiveHalf(false)
  , change_stamp(0)
  , machine_state(game_state_data, game_state_data)
  , number_of_yellow_goals(0)
  , number_of_blue_goals(0)
{
  // STATES
  machine_state.add_state(state_name::stop,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::stop);
                          });
  machine_state.add_state(state_name::running,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::running);
                          });
  machine_state.add_state(state_name::halt,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::halt);
                          });
  machine_state.add_state(state_name::free_kick,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::free_kick);
                          });
  machine_state.add_state(state_name::kickoff,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::kickoff);
                          });
  machine_state.add_state(state_name::prepare_kickoff,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::prepare_kickoff);
                          });
  machine_state.add_state(state_name::penalty,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::penalty);
                          });
  machine_state.add_init_state(state_name::halt);

  // EDGES
  machine_state.add_edge(edge_name::force_start, state_name::stop, state_name::running,
                         command_is_<SSL_Referee::FORCE_START>);

  machine_state.add_edge(edge_name::running_to_stop, state_name::running, state_name::stop,
                         command_is_<SSL_Referee::STOP>);

  machine_state.add_edge(edge_name::stop_to_prepare_kickoff, state_name::stop, state_name::prepare_kickoff,
                         command_is_one_of_<SSL_Referee::PREPARE_KICKOFF_BLUE, SSL_Referee::PREPARE_KICKOFF_YELLOW>,
                         [&](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           if ((game_state_data.current().command() == SSL_Referee::PREPARE_KICKOFF_YELLOW))
                           {
                             team_having_kickoff = Ai::Yellow;
                           }
                           else
                           {
                             team_having_kickoff = Ai::Blue;
                           }
                         });

  machine_state.add_edge(edge_name::prepare_kickoff_to_stop, state_name::prepare_kickoff, state_name::stop,
                         command_is_<SSL_Referee::STOP>);

  machine_state.add_edge(edge_name::kickoff_to_stop, state_name::kickoff, state_name::stop,
                         command_is_<SSL_Referee::STOP>);

  machine_state.add_edge(edge_name::start, state_name::prepare_kickoff, state_name::kickoff,
                         command_is_<SSL_Referee::NORMAL_START>);

  machine_state.add_edge(edge_name::ball_move_after_kickoff, state_name::kickoff, state_name::running,
                         [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           return (ball_is_moving() &&
                                   not(command_is_<SSL_Referee::HALT>(data, run_number, atomic_run_number)) &&
                                   not(command_is_<SSL_Referee::STOP>(data, run_number, atomic_run_number)));
                         });

  machine_state.add_edge(edge_name::prepare_kickoff_to_halt, state_name::prepare_kickoff, state_name::halt,
                         command_is_<SSL_Referee::HALT>);

  machine_state.add_edge(edge_name::kickoff_to_halt, state_name::kickoff, state_name::halt,
                         command_is_<SSL_Referee::HALT>);

  machine_state.add_edge(edge_name::stop_to_free_kick, state_name::stop, state_name::free_kick,
                         command_is_one_of_4_<SSL_Referee::DIRECT_FREE_YELLOW, SSL_Referee::DIRECT_FREE_BLUE,
                                              SSL_Referee::INDIRECT_FREE_YELLOW, SSL_Referee::INDIRECT_FREE_BLUE>,
                         [&](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           if ((game_state_data.current().command() == SSL_Referee::DIRECT_FREE_YELLOW) or
                               (game_state_data.current().command() == SSL_Referee::INDIRECT_FREE_YELLOW))
                           {
                             team_having_free_kick = Ai::Yellow;
                           }
                           else
                           {
                             team_having_free_kick = Ai::Blue;
                           }
                           if ((game_state_data.current().command() == SSL_Referee::DIRECT_FREE_YELLOW) or
                               (game_state_data.current().command() == SSL_Referee::DIRECT_FREE_BLUE))
                           {
                             free_kick_type = DIRECT;
                           }
                           else
                           {
                             free_kick_type = INDIRECT;
                           }
                         });

  machine_state.add_edge(edge_name::free_kick_to_stop, state_name::free_kick, state_name::stop,
                         command_is_<SSL_Referee::STOP>);

  machine_state.add_edge(edge_name::ball_move_after_free_kick, state_name::free_kick, state_name::running,
                         [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           return (ball_is_moving() &&
                                   not(command_is_<SSL_Referee::HALT>(data, run_number, atomic_run_number)) &&
                                   not(command_is_<SSL_Referee::STOP>(data, run_number, atomic_run_number)));
                         });

  machine_state.add_edge(edge_name::free_kick_to_halt, state_name::free_kick, state_name::halt,
                         command_is_<SSL_Referee::HALT>);

  machine_state.add_edge(edge_name::stop_to_penalty, state_name::stop, state_name::penalty,
                         command_is_one_of_<SSL_Referee::PREPARE_PENALTY_YELLOW, SSL_Referee::PREPARE_PENALTY_BLUE>,
                         [&](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           if ((game_state_data.current().command() == SSL_Referee::PREPARE_PENALTY_YELLOW))
                           {
                             team_having_penalty = Ai::Yellow;
                           }
                           else
                           {
                             team_having_penalty = Ai::Blue;
                           }
                         });

  machine_state.add_edge(edge_name::penalty_to_stop, state_name::penalty, state_name::stop,
                         command_is_<SSL_Referee::STOP>);

  machine_state.add_edge(edge_name::ball_move_after_penalty, state_name::penalty, state_name::running,
                         [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           return (ball_is_moving() &&
                                   not(command_is_<SSL_Referee::HALT>(data, run_number, atomic_run_number)) &&
                                   not(command_is_<SSL_Referee::STOP>(data, run_number, atomic_run_number)));
                         });

  machine_state.add_edge(edge_name::stop_to_halt, state_name::stop, state_name::halt, command_is_<SSL_Referee::HALT>);

  machine_state.add_edge(edge_name::halt_to_stop, state_name::halt, state_name::stop, command_is_<SSL_Referee::STOP>);

  machine_state.add_edge(edge_name::running_to_halt, state_name::running, state_name::halt,
                         command_is_<SSL_Referee::HALT>);

  machine_state.add_edge(edge_name::goal, state_name::stop, state_name::stop,
                         command_is_one_of_<SSL_Referee::GOAL_YELLOW, SSL_Referee::GOAL_BLUE>,
                         [&](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           if ((game_state_data.current().command() == SSL_Referee::GOAL_YELLOW))
                           {
                             number_of_yellow_goals += 1;
                           }
                           else
                           {
                             number_of_blue_goals += 1;
                           }
                         }

  );

  machine_state.add_edge(edge_name::penalty_to_halt, state_name::penalty, state_name::halt,
                         command_is_<SSL_Referee::HALT>);

  machine_state.execute_at_each_edge([&](std::string edge_id, GameStateData& state_data, GameStateData& edge_data,
                                         unsigned int run_number,
                                         unsigned int atomic_run_number) { change_stamp += 1; });

  machine_state.export_to_file("/tmp/game_state.dot");

  machine_state.start();
}

bool GameState::ball_is_moving()
{
  Vector2d ball_velocity = ai_data.ball.get_movement().linear_velocity(ai_data.time);
  double threshold = 0.001;
  if (std::abs(ball_velocity[0]) + std::abs(ball_velocity[1]) > 0 + threshold)
  {
    return true;
  }
  return false;
}

void GameState::extract_data()
{
  SSL_Referee data = referee.getData();
  // DEBUG("SSL REFEREE PROTOBUF : " << data);
  // Use this function just one time if you want to avoir thread
  // issue.
  if (game_state_data.last_time < data.packet_timestamp())
  {
    game_state_data.datas.insert(data);
    if (data.has_blueteamonpositivehalf())
    {
      blueTeamOnPositiveHalf = data.blueteamonpositivehalf();
    }
  }
}

void GameState::save_last_time_stamps()
{
  if (game_state_data.last_time < game_state_data.current().packet_timestamp())
  {
    game_state_data.last_time = game_state_data.current().packet_timestamp();

    if (game_state_data.last_command_counter < game_state_data.current().command_counter())
    {
      game_state_data.last_command_counter = game_state_data.current().command_counter();
      DEBUG("Command is : " << game_state_data.current().command());
      DEBUG("State is : " << machine_state.current_states());
    }
  }
}

void GameState::update(double time)
{
  extract_data();
  machine_state.run();
  assert(machine_state.current_states().size() == 1);
  save_last_time_stamps();
}

unsigned int GameState::get_change_stamp() const
{
  return change_stamp;
}
const GameState::ID& GameState::get_state() const
{
  assert(machine_state.current_states().size() == 1);
  return *(machine_state.current_states().begin());
}

Ai::Team GameState::kickoff_team() const
{
  return team_having_kickoff;
}

Ai::Team GameState::penalty_team() const
{
  return team_having_penalty;
}

Ai::Team GameState::free_kick_team() const
{
  return team_having_free_kick;
}

free_kick_type_id GameState::type_of_the_free_kick() const
{
  return free_kick_type;
}

bool GameState::blue_have_it_s_goal_on_positive_x_axis() const
{
  return blueTeamOnPositiveHalf;
}

int GameState::yellow_goalie_id() const
{
  return game_state_data.current().yellow().goalie();
}

int GameState::blue_goalie_id() const
{
  return game_state_data.current().blue().goalie();
}

bool GameState::state_is_newer(unsigned int last_change_stamp) const
{
  return change_stamp > last_change_stamp;
}

Ai::Team GameState::get_team_color(const std::string& team_name) const
{
  if (game_state_data.current().yellow().name() == team_name)
  {
    return Ai::Yellow;
  }
  if (game_state_data.current().blue().name() == team_name)
  {
    return Ai::Blue;
  }
  return Ai::Unknown;
};

RefereeClient& GameState::getRefereeClient()
{
  return referee;
}

}  // namespace rhoban_ssl
