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
#include <game_informations.h>

namespace rhoban_ssl
{
GameStateData::GameStateData() : datas(2), last_time(0.0), last_command_counter(0)
{
}

const Referee& GameStateData::current() const
{
  return datas[0];
}

const Referee& GameStateData::old() const
{
  return datas[1];
}

bool GameStateData::commandIsNew() const
{
  return (last_time < current().packet_timestamp()) and (last_command_counter < current().command_counter());
}

template <Referee_Command E>
bool command_is_(const GameStateData& game_state_data, unsigned int run_number, unsigned int atomic_run_number)
{
  return (game_state_data.commandIsNew()) and (game_state_data.current().command() == E);
}

template <Referee_Command E1, Referee_Command E2>
bool command_is_one_of_(const GameStateData& game_state_data, unsigned int run_number, unsigned int atomic_run_number)
{
  return (game_state_data.commandIsNew()) and
         ((game_state_data.current().command() == E1) or (game_state_data.current().command() == E2));
}

template <Referee_Command E1, Referee_Command E2, Referee_Command E3, Referee_Command E4>
bool command_is_one_of_4_(const GameStateData& game_state_data, unsigned int run_number, unsigned int atomic_run_number)
{
  return (game_state_data.commandIsNew()) and
         ((game_state_data.current().command() == E1) or (game_state_data.current().command() == E2) or
          (game_state_data.current().command() == E3) or (game_state_data.current().command() == E4));
}

GameState::GameState()
  : blueTeamOnPositiveHalf_(false)
  , change_stamp_(0)
  , machine_state_(game_state_data_, game_state_data_)
  , number_of_yellow_goals_(0)
  , number_of_blue_goals_(0)
{
  // STATES
  machine_state_.addState(state_name::stop,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::stop);
                          });
  machine_state_.addState(state_name::running,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::running);
                          });
  machine_state_.addState(state_name::halt,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::halt);
                          });
  machine_state_.addState(state_name::free_kick,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::free_kick);
                          });
  machine_state_.addState(state_name::kickoff,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::kickoff);
                          });
  machine_state_.addState(state_name::prepare_kickoff,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::prepare_kickoff);
                          });
  machine_state_.addState(state_name::penalty,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::penalty);
                          });
  machine_state_.addState(state_name::prepare_penalty,
                          [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                            // DEBUG(state_name::prepare_penalty);
                          });

  machine_state_.addInitState(state_name::halt);

  // EDGES
  machine_state_.addEdge(edge_name::force_start, state_name::stop, state_name::running,
                         command_is_<Referee::FORCE_START>);

  machine_state_.addEdge(edge_name::running_to_stop, state_name::running, state_name::stop, command_is_<Referee::STOP>);

  machine_state_.addEdge(edge_name::stop_to_prepare_kickoff, state_name::stop, state_name::prepare_kickoff,
                         command_is_one_of_<Referee::PREPARE_KICKOFF_BLUE, Referee::PREPARE_KICKOFF_YELLOW>,
                         [&](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           if ((game_state_data_.current().command() == Referee::PREPARE_KICKOFF_YELLOW))
                           {
                             team_having_kickoff_ = (ai::Config::we_are_blue) ? Opponent : Ally;
                           }
                           else
                           {
                             team_having_kickoff_ = (ai::Config::we_are_blue) ? Ally : Opponent;
                           }
                         });

  machine_state_.addEdge(edge_name::prepare_kickoff_to_stop, state_name::prepare_kickoff, state_name::stop,
                         command_is_<Referee::STOP>);

  machine_state_.addEdge(edge_name::kickoff_to_stop, state_name::kickoff, state_name::stop, command_is_<Referee::STOP>);

  machine_state_.addEdge(edge_name::start, state_name::prepare_kickoff, state_name::kickoff,
                         command_is_<Referee::NORMAL_START>);

  machine_state_.addEdge(edge_name::ball_move_after_kickoff, state_name::kickoff, state_name::running,
                         [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           return (ballIsMoving() &&
                                   not(command_is_<Referee::HALT>(data, run_number, atomic_run_number)) &&
                                   not(command_is_<Referee::STOP>(data, run_number, atomic_run_number)));
                         });

  machine_state_.addEdge(edge_name::prepare_kickoff_to_halt, state_name::prepare_kickoff, state_name::halt,
                         command_is_<Referee::HALT>);

  machine_state_.addEdge(edge_name::kickoff_to_halt, state_name::kickoff, state_name::halt, command_is_<Referee::HALT>);

  machine_state_.addEdge(edge_name::stop_to_free_kick, state_name::stop, state_name::free_kick,
                         command_is_one_of_4_<Referee::DIRECT_FREE_YELLOW, Referee::DIRECT_FREE_BLUE,
                                              Referee::INDIRECT_FREE_YELLOW, Referee::INDIRECT_FREE_BLUE>,
                         [&](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           if ((game_state_data_.current().command() == Referee::DIRECT_FREE_YELLOW) or
                               (game_state_data_.current().command() == Referee::INDIRECT_FREE_YELLOW))
                           {
                             team_having_free_kick_ = (ai::Config::we_are_blue) ? Opponent : Ally;
                           }
                           else
                           {
                             team_having_free_kick_ = (ai::Config::we_are_blue) ? Ally : Opponent;
                           }
                           if ((game_state_data_.current().command() == Referee::DIRECT_FREE_YELLOW) or
                               (game_state_data_.current().command() == Referee::DIRECT_FREE_BLUE))
                           {
                             free_kick_type_ = DIRECT;
                           }
                           else
                           {
                             free_kick_type_ = INDIRECT;
                           }
                         });

  machine_state_.addEdge(edge_name::free_kick_to_stop, state_name::free_kick, state_name::stop,
                         command_is_<Referee::STOP>);

  machine_state_.addEdge(edge_name::ball_move_after_free_kick, state_name::free_kick, state_name::running,
                         [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           return (ballIsMoving() &&
                                   not(command_is_<Referee::HALT>(data, run_number, atomic_run_number)) &&
                                   not(command_is_<Referee::STOP>(data, run_number, atomic_run_number)));
                         });

  machine_state_.addEdge(edge_name::free_kick_to_halt, state_name::free_kick, state_name::halt,
                         command_is_<Referee::HALT>);

  machine_state_.addEdge(edge_name::stop_to_prepare_penalty, state_name::stop, state_name::prepare_penalty,
                         command_is_one_of_<Referee::PREPARE_PENALTY_YELLOW, Referee::PREPARE_PENALTY_BLUE>,
                         [&](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           if ((game_state_data_.current().command() == Referee::PREPARE_PENALTY_YELLOW))
                           {
                             team_having_penalty_ = (ai::Config::we_are_blue) ? Opponent : Ally;
                           }
                           else
                           {
                             team_having_penalty_ = (ai::Config::we_are_blue) ? Ally : Opponent;
                           }
                         });

  machine_state_.addEdge(edge_name::prepare_penalty_to_stop, state_name::prepare_penalty, state_name::stop,
                         command_is_<Referee::STOP>);

  machine_state_.addEdge(edge_name::penalty_to_stop, state_name::penalty, state_name::stop, command_is_<Referee::STOP>);

  machine_state_.addEdge(edge_name::start_penalty, state_name::prepare_penalty, state_name::penalty,
                         command_is_<Referee::NORMAL_START>);

  machine_state_.addEdge(edge_name::ball_move_after_penalty, state_name::penalty, state_name::running,
                         [this](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           return (ballIsMoving() &&
                                   not(command_is_<Referee::HALT>(data, run_number, atomic_run_number)) &&
                                   not(command_is_<Referee::STOP>(data, run_number, atomic_run_number)));
                         });

  machine_state_.addEdge(edge_name::prepare_penalty_to_halt, state_name::prepare_penalty, state_name::halt,
                         command_is_<Referee::HALT>);

  machine_state_.addEdge(edge_name::penalty_to_halt, state_name::penalty, state_name::halt, command_is_<Referee::HALT>);

  machine_state_.addEdge(edge_name::stop_to_halt, state_name::stop, state_name::halt, command_is_<Referee::HALT>);

  machine_state_.addEdge(edge_name::halt_to_stop, state_name::halt, state_name::stop, command_is_<Referee::STOP>);

  machine_state_.addEdge(edge_name::running_to_halt, state_name::running, state_name::halt, command_is_<Referee::HALT>);

  machine_state_.addEdge(edge_name::goal, state_name::stop, state_name::stop,
                         command_is_one_of_<Referee::GOAL_YELLOW, Referee::GOAL_BLUE>,
                         [&](const GameStateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                           if ((game_state_data_.current().command() == Referee::GOAL_YELLOW))
                           {
                             number_of_yellow_goals_ += 1;
                           }
                           else
                           {
                             number_of_blue_goals_ += 1;
                           }
                         }

  );

  machine_state_.addEdge(edge_name::penalty_to_halt, state_name::penalty, state_name::halt, command_is_<Referee::HALT>);

  machine_state_.executeAtEachEdge([&](std::string edge_id, GameStateData& state_data, GameStateData& edge_data,
                                       unsigned int run_number,
                                       unsigned int atomic_run_number) { change_stamp_ += 1; });

  machine_state_.exportToFile("/tmp/game_state.dot");

  machine_state_.start();
}

bool GameState::ballIsMoving()
{
  Vector2d ball_velocity = Data::get()->ball.getMovement().linearVelocity(Data::get()->ai_data.time);
  double threshold = 0.001;
  if (std::abs(ball_velocity[0]) + std::abs(ball_velocity[1]) > 0 + threshold)
  {
    return true;
  }
  return false;
}

void GameState::extractData(const Referee& new_data)
{
  // DEBUG("SSL REFEREE PROTOBUF : " << data.stage_time_left());
  // Use this function just one time if you want to avoir thread
  // issue.
  if (game_state_data_.last_time < new_data.packet_timestamp())
  {
    game_state_data_.datas.insert(new_data);
    if (new_data.has_blue_team_on_positive_half())
    {
      blueTeamOnPositiveHalf_ = new_data.has_blue_team_on_positive_half();
    }
  }
}

void GameState::saveLastTimeStamps()
{
  if (game_state_data_.last_time < game_state_data_.current().packet_timestamp())
  {
    game_state_data_.last_time = game_state_data_.current().packet_timestamp();

    if (game_state_data_.last_command_counter < game_state_data_.current().command_counter())
    {
      game_state_data_.last_command_counter = game_state_data_.current().command_counter();
      DEBUG("Command is : " << game_state_data_.current().command());
      DEBUG("State is : " << machine_state_.currentStates());
    }
  }
}

void GameState::update(const Referee& new_referee)
{
  extractData(new_referee);
  machine_state_.run();
  assert(machine_state_.currentStates().size() == 1);
  saveLastTimeStamps();
}

unsigned int GameState::getChangeStamp() const
{
  return change_stamp_;
}
const GameState::ID& GameState::getState() const
{
  assert(machine_state_.currentStates().size() == 1);
  return *(machine_state_.currentStates().begin());
}

Team GameState::kickoffTeam() const
{
  return team_having_kickoff_;
}

Team GameState::penaltyTeam() const
{
  return team_having_penalty_;
}

Team GameState::freeKickTeam() const
{
  return team_having_free_kick_;
}

free_kick_type_id GameState::typeOfTheFreeKick() const
{
  return free_kick_type_;
}

bool GameState::blueHaveItsGoalOnPositiveXAxis() const
{
  return blueTeamOnPositiveHalf_;
}

int GameState::yellowGoalieId() const
{
  return int(game_state_data_.current().yellow().goalkeeper());
}

int GameState::blueGoalieId() const
{
  return int(game_state_data_.current().blue().goalkeeper());
}

bool GameState::stateIsNewer(unsigned int last_change_stamp) const
{
  return change_stamp_ > last_change_stamp;
}

}  // namespace rhoban_ssl
