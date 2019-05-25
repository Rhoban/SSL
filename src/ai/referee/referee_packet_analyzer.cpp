#include "referee_packet_analyzer.h"
#include "referee_client_single_thread.h"
#include "data.h"

namespace rhoban_ssl
{
namespace referee
{
RefereePacketAnalyzer::RefereePacketAnalyzer()
{
}

bool RefereePacketAnalyzer::runTask()
{
  if (/*referee::*/ RefereeMessages::singleton_.last_packets_.size() > 0)
  {
    ::Referee* data = /*referee::*/ RefereeMessages::singleton_.last_packets_.front();
    GlobalDataSingleThread::singleton_.referee_.game_state.update(*data);

    if (data->packet_timestamp() > GlobalDataSingleThread::singleton_.referee_.packet_timestamp)
    {
      // update game state
      GlobalDataSingleThread::singleton_.referee_.packet_timestamp = data->packet_timestamp();

      GlobalDataSingleThread::singleton_.referee_.state_changed =
          (data->stage() != GlobalDataSingleThread::singleton_.referee_.current_stage);

      GlobalDataSingleThread::singleton_.referee_.current_stage = data->stage();
      GlobalDataSingleThread::singleton_.referee_.stage_time_left = data->stage_time_left();
      GlobalDataSingleThread::singleton_.referee_.command_timestamp = data->command_timestamp();
      GlobalDataSingleThread::singleton_.referee_.current_command = data->command();

      if (data->has_next_command())
        GlobalDataSingleThread::singleton_.referee_.next_command = data->next_command();

      // Update teams infos
      if (ai::Config::we_are_blue)
      {
        updateTeamInfo(Ally, data->blue());
        updateTeamInfo(Opponent, data->yellow());
      }
      else
      {
        updateTeamInfo(Ally, data->yellow());
        updateTeamInfo(Opponent, data->blue());
      }

      // Update side information
      if (data->has_blue_team_on_positive_half())
      {
        GlobalDataSingleThread::singleton_.referee_.blue_team_on_positive_half = data->blue_team_on_positive_half();
      }
    }
  }
  return true;
}

void RefereePacketAnalyzer::updateTeamInfo(Team team, const Referee_TeamInfo& new_infos)
{
  data::Referee::TeamInfo& team_infos_to_update = GlobalDataSingleThread::singleton_.referee_.teams_info[team];

  team_infos_to_update.name = new_infos.name();
  team_infos_to_update.score = new_infos.score();
  team_infos_to_update.red_cards_count = new_infos.red_cards();
  team_infos_to_update.yellow_cards_count = new_infos.yellow_cards();

  team_infos_to_update.yellow_card_times.clear();
  for (auto yellow_card_time : new_infos.yellow_card_times())
  {
    team_infos_to_update.yellow_card_times.push_back(yellow_card_time);
  }

  team_infos_to_update.timeout_remaining_count = new_infos.timeouts();
  team_infos_to_update.timeout_remaining_time = new_infos.timeout_time();

  if (new_infos.goalkeeper() != team_infos_to_update.goalkeeper_number)
  {
    if (new_infos.goalkeeper() >= ai::Config::NB_OF_ROBOTS_BY_TEAM)
    {
      std::cerr << "ERROR : invalid goalkeeper number from referee !!!" << std::endl;
    }
    else
    {
      // Update goalie number in the information of the robots concerned
      GlobalDataSingleThread::singleton_.robots_[team][team_infos_to_update.goalkeeper_number].is_goalie = false;
      team_infos_to_update.goalkeeper_number = new_infos.goalkeeper();
      GlobalDataSingleThread::singleton_.robots_[team][team_infos_to_update.goalkeeper_number].is_goalie = true;
    }
  }

  if (new_infos.has_foul_counter())
    team_infos_to_update.foul_counter = int(new_infos.foul_counter());

  if (new_infos.has_ball_placement_failures())
    team_infos_to_update.ball_placement_failures = int(new_infos.ball_placement_failures());

  if (new_infos.has_can_place_ball())
    team_infos_to_update.can_place_ball = new_infos.can_place_ball();

  if (new_infos.has_max_allowed_bots())
    team_infos_to_update.max_allowed_bots = int(new_infos.max_allowed_bots());
}

///////////////////////////////////////////////////////////////////////////////

RefereeTerminalPrinter::RefereeTerminalPrinter() : counter_(0)
{
}

bool RefereeTerminalPrinter::runTask()
{
  counter_ += 1;
  std::stringstream ss;

  // ss << "\033[2J\033[1;1H";  // this clear the terminal
  ss << "--------------------------------------------------" << std::endl;
  ss << "Refere infos (" << counter_ << ")" << std::endl;
  ss << "--------------------------------------------------" << std::endl;
  ss << "stage: " << GlobalDataSingleThread::singleton_.referee_.getCurrentStageName() << std::endl;
  ss << "stage_time_left: " << GlobalDataSingleThread::singleton_.referee_.stage_time_left << std::endl;
  ss << "state: " << GlobalDataSingleThread::singleton_.referee_.getCurrentStateName() << std::endl;
  ss << "command_timestamp: " << GlobalDataSingleThread::singleton_.referee_.command_timestamp << std::endl;
  ss << "remaining time: " << GlobalDataSingleThread::singleton_.referee_.stage_time_left << std::endl;

  for (int i = 0; i < 2; ++i)
  {
    ss << "----- TEAM : " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].name << " ----- " << std::endl;
    ss << "score: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].score << std::endl;
    ss << "red_cards: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].red_cards_count << std::endl;
    ss << "yellow_cards: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].yellow_cards_count << std::endl;
    ss << "  timeouts: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].timeout_remaining_count
       << std::endl;
    ss << "  timeout_time: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].timeout_remaining_time
       << std::endl;
    ss << "  goalie: " << GlobalDataSingleThread::singleton_.referee_.teams_info[i].goalkeeper_number << std::endl;
  }
  std::cout << ss.str();
  std::cout << std::flush;
  return true;
}

}  // namespace referee
}  // namespace rhoban_ssl
