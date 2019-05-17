#include "referee.h"

namespace rhoban_ssl
{
namespace data
{
Referee::TeamInfo::TeamInfo()
  : name("")
  , score(0)
  , available_timeout_count(0)
  , available_time_of_timeout_(0)
  , goalkeeper_number(0)
  , red_cards_count(0)
  , yellow_cards_count(0)
  , yellow_card_times()
  , foul_counter(-1)
  , ball_placement_failures(-1)
  , can_place_ball(-1)
  , max_allowed_bots(-1)

{
}

///////////////////////////////////////////////////////////////////////////////

Referee::Referee() : state_changed(true), blue_team_on_positive_half(false), packet_timestamp(0)
{
}

std::string Referee::getCurrentStageName()
{
  return ::Referee::Stage_Name(current_stage);
}

std::string Referee::getCurrentStateName()
{
  return ::Referee::Command_Name(current_command);
}

std::string Referee::getNextStateName()
{
  return ::Referee::Command_Name(next_command);
}

}  // namespace data
}  // namespace rhoban_ssl
