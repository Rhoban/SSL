/*
    This file is part of SSL.

    Copyright 2019 xavier.mlr@live.fr

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

#include <execution_manager.h>
#include <ssl_referee.pb.h>
#include <referee/game_state.h>

namespace rhoban_ssl
{
namespace data
{
class Referee
{
public:
  /**
   * @brief Old machine state
   * Need to be update but works.
   */
  GameState game_state;

  /**
   * @brief The TeamInfo struct
   * Contains all Referee_TeamInfo data
   * @see ssl_referee.proto
   */
  struct TeamInfo
  {
    std::string name;
    uint score;
    uint available_timeout_count;
    uint available_time_of_timeout_;
    uint goalkeeper_number;
    uint red_cards_count;
    uint yellow_cards_count;
    std::vector<uint> yellow_card_times;

    // optional
    int foul_counter;
    int ball_placement_failures;
    int can_place_ball;
    int max_allowed_bots;

    TeamInfo();
  };

  Referee();

  /**
   * @brief Current stage of the referee.
   * These are the "coarse" stages of the game.
   * @see ssl_referee.proto
   */
  Referee_Stage current_stage;

  /**
   * @brief The number of microseconds left in the stage.
   *
   * The following stages have this value; the rest do not:
   * NORMAL_FIRST_HALF
   * NORMAL_HALF_TIME
   * NORMAL_SECOND_HALF
   * EXTRA_TIME_BREAK
   * EXTRA_FIRST_HALF
   * EXTRA_HALF_TIME
   * EXTRA_SECOND_HALF
   * PENALTY_SHOOTOUT_BREAK
   *
   * If the stage runs over its specified time, this value
   * becomes negative.
   * @see ssl_referee.proto
   */
  int stage_time_left;

  /**
   * @brief Current state of the referee.
   *
   * These are the "fine" states of play on the field.
   * @see ssl_referee.proto
   */
  Referee_Command current_command;

  /**
   * @brief The next_state correspond to command that will be issued
   * after the current stoppage and ball placement to continue the game.
   */
  Referee_Command next_command;

  /**
   * @brief state_changed
   */
  bool state_changed;

  /**
   * @brief command_timestamp
   *
   * The UNIX timestamp when the command was issued, in microseconds.
   * This value changes only when a new command is issued, not on each packet.
   * @see ssl_referee.proto
   */
  ulong command_timestamp;

  /**
   * @brief teams_info contains the info of the Ally at 0 and Opponent at 1
   */
  TeamInfo teams_info[2];

  /**
   * @brief blue_team_on_positive_half
   *
   * Information about the direction of play.
   * True, if the blue team will have it's goal on the positive x-axis of the ssl-vision coordinate system.
   * Obviously, the yellow team will play on the opposite half.
   * @see ssl_referee.proto
   */
  bool blue_team_on_positive_half;

  ulong packet_timestamp;

  std::string getCurrentStageName();
  std::string getCurrentStateName();
  std::string getNextStateName();
};

}  // namespace data
}  // namespace rhoban_ssl
