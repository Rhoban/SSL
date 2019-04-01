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

#pragma once

#include <RefereeClient.h>
#include <core/machine_state.h>
#include <math/circular_vector.h>
#include <ai_data.h>

namespace RhobanSSL
{
struct state_name
{
  static const constexpr char* stop = "stop";
  static const constexpr char* running = "running";
  static const constexpr char* halt = "halt";
  static const constexpr char* free_kick = "free_kick";
  static const constexpr char* kickoff = "kickoff";
  static const constexpr char* prepare_kickoff = "prepare_kickoff";
  static const constexpr char* penalty = "penalty";
};

struct edge_name
{
  static const constexpr char* force_start = "force_start";
  static const constexpr char* running_to_stop = "running_to_stop";

  static const constexpr char* stop_to_prepare_kickoff = "stop_to_prepare_kickoff";
  static const constexpr char* prepare_kickoff_to_stop = "prepare_kickoff_to_stop";
  static const constexpr char* kickoff_to_stop = "kickoff_to_stop";
  static const constexpr char* start = "start";
  static const constexpr char* ball_move_after_kickoff = "ball_move_after_kickoff";
  static const constexpr char* prepare_kickoff_to_halt = "prepare_kickoff_to_halt";
  static const constexpr char* kickoff_to_halt = "kickoff_to_halt";

  static const constexpr char* stop_to_free_kick = "stop_to_free_kick";
  static const constexpr char* free_kick_to_stop = "free_kick_to_stop";
  static const constexpr char* ball_move_after_free_kick = "ball_move_after_free_kick";
  static const constexpr char* free_kick_to_halt = "free_kick_to_halt";

  static const constexpr char* stop_to_penalty = "stop_to_penalty";
  static const constexpr char* penalty_to_stop = "penalty_to_stop";
  static const constexpr char* ball_move_after_penalty = "ball_move_after_penalty";
  static const constexpr char* penalty_to_halt = "penalty_to_halt";

  static const constexpr char* stop_to_halt = "stop_to_halt";
  static const constexpr char* halt_to_stop = "halt_to_stop";

  static const constexpr char* running_to_halt = "running_to_halt";

  static const constexpr char* goal = "goal";
};

typedef enum free_kick_type_id
{
  UNKNOWN,
  DIRECT,
  INDIRECT
} free_kick_type_id;

struct GameStateData
{
  // datas[0] is the most recent
  // datas[1] the older
  circular_vector<SSL_Referee> datas;

  double last_time;
  uint32_t last_command_counter;

  GameStateData();

  const SSL_Referee& current() const;
  const SSL_Referee& old() const;

  bool command_is_new() const;
};

class GameState
{
private:
  Ai::AiData& ai_data;
  bool blueTeamOnPositiveHalf;

  RefereeClient referee;
  GameStateData game_state_data;
  unsigned int change_stamp;

  typedef std::string ID;
  typedef construct_machine_state_infrastructure<ID, GameStateData, GameStateData> machine_infrastructure;

  machine_infrastructure::MachineState machine_state;

  bool ball_is_moving();
  void extract_data();
  void save_last_time_stamps();

  Ai::Team team_having_kickoff;
  Ai::Team team_having_penalty;
  Ai::Team team_having_free_kick;
  free_kick_type_id free_kick_type;
  int number_of_yellow_goals;
  int number_of_blue_goals;

public:
  GameState(Ai::AiData& ai_data);

  unsigned int get_change_stamp() const;
  const ID& get_state() const;

  void update(double time);

  Ai::Team kickoff_team() const;
  Ai::Team penalty_team() const;
  Ai::Team free_kick_team() const;

  free_kick_type_id type_of_the_free_kick() const;

  bool blue_have_it_s_goal_on_positive_x_axis() const;
  Ai::Team get_team_color(const std::string& team_name) const;

  int yellow_goalie_id() const;
  int blue_goalie_id() const;

  bool state_is_newer(unsigned int last_change_stamp) const;

  RefereeClient& getRefereeClient();
};

}  // namespace RhobanSSL
