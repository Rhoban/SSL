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
#include <config.h>

namespace rhoban_ssl
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
  static const constexpr char* prepare_penalty = "prepare_penalty";
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

  static const constexpr char* stop_to_prepare_penalty = "stop_to_prepare_penalty";
  static const constexpr char* prepare_penalty_to_stop = "prepare_penalty_to_stop";
  static const constexpr char* penalty_to_stop = "penalty_to_stop";
  static const constexpr char* start_penalty = "start_penalty";
  static const constexpr char* ball_move_after_penalty = "ball_move_after_penalty";
  static const constexpr char* prepare_penalty_to_halt = "prepare_penalty_to_halt";
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
  CircularVector<Referee> datas;

  double last_time;
  uint32_t last_command_counter;

  GameStateData();

  const Referee& current() const;
  const Referee& old() const;

  bool commandIsNew() const;
};

class GameState
{
  bool blueTeamOnPositiveHalf_;

  GameStateData game_state_data_;
  unsigned int change_stamp_;

  typedef std::string ID;
  typedef construct_machine_state_infrastructure<ID, GameStateData, GameStateData> MachineInfrastructure;

  MachineInfrastructure::MachineState machine_state_;

  bool ballIsMoving();
  void extractData(const Referee& new_data);
  void saveLastTimeStamps();

  Team team_having_kickoff_;
  Team team_having_penalty_;
  Team team_having_free_kick_;
  free_kick_type_id free_kick_type_;
  int number_of_yellow_goals_;
  int number_of_blue_goals_;

public:
  GameState();

  unsigned int getChangeStamp() const;
  const ID& getState() const;

  void update(const Referee& new_referee);

  Team kickoffTeam() const;
  Team penaltyTeam() const;
  Team freeKickTeam() const;

  free_kick_type_id typeOfTheFreeKick() const;

  bool blueHaveItsGoalOnPositiveXAxis() const;

  int yellowGoalieId() const;
  int blueGoalieId() const;

  bool stateIsNewer(unsigned int last_change_stamp) const;
};

}  // namespace rhoban_ssl
