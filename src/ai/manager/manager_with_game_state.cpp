#include "manager_with_game_state.h"

namespace rhoban_ssl
{
namespace manager
{
ManagerWithGameState::ManagerWithGameState(ai::AiData& ai_data, const GameState& game_state)
  : Manager(ai_data), game_state(game_state), last_change_stamp(0)
{
}

void ManagerWithGameState::analyse_data(double time)
{
  // We change the point of view of the team
  change_team_and_point_of_view(game_state.getTeamColor(get_team_name()),
                                game_state.blueHaveItsGoalOnPositiveXAxis());
  change_ally_and_opponent_goalie_id(game_state.blueGoalieId(), game_state.yellowGoalieId());
}

void ManagerWithGameState::choose_a_strategy(double time)
{
  if (game_state.stateIsNewer(last_change_stamp))
  {
    clear_strategy_assignement();
    last_change_stamp = game_state.getChangeStamp();
    if (game_state.getState() == state_name::running)
    {
      DEBUG("RUNNING");
      start_running();
    }
    else if (game_state.getState() == state_name::stop)
    {
      DEBUG("STOP");
      start_stop();
    }
    else if (game_state.getState() == state_name::halt)
    {
      DEBUG("HALT");
      start_halt();
    }
    else if (game_state.getState() == state_name::free_kick)
    {
      free_kick_type_id type_free_kick = game_state.typeOfTheFreeKick();
      ai::Team team_free_kick = game_state.freeKickTeam();
      if (type_free_kick == DIRECT and team_free_kick == get_team())
      {
        DEBUG("DIRECT KICK ALLY");
        start_direct_kick_ally();
      }
      else if (type_free_kick == DIRECT and team_free_kick != get_team())
      {
        DEBUG("DIRECT KICK OPPONENT");
        start_direct_kick_opponent();
      }
      else if (type_free_kick == INDIRECT and team_free_kick == get_team())
      {
        DEBUG("INDIRECT KICK ALLY");
        start_indirect_kick_ally();
      }
      else if (type_free_kick == INDIRECT and team_free_kick != get_team())
      {
        DEBUG("INDIRECT KICK OPPONENT");
        start_indirect_kick_opponent();
      }
    }
    else if (game_state.getState() == state_name::prepare_kickoff)
    {
      ai::Team team_prepare_kickoff = game_state.kickoffTeam();
      if (team_prepare_kickoff == get_team())
      {
        DEBUG("PREPARE KICKOFF ALLY");
        start_prepare_kickoff_ally();
      }
      else
      {
        DEBUG("PREPARE KICKOFF OPPONENT");
        start_prepare_kickoff_opponent();
      }
    }
    else if (game_state.getState() == state_name::kickoff)
    {
      ai::Team team_kickoff = game_state.kickoffTeam();
      if (team_kickoff == get_team())
      {
        DEBUG("KICKOFF ALLY");
        start_kickoff_ally();
      }
      else
      {
        DEBUG("KICKOFF OPPONENT");
        start_kickoff_opponent();
      }
    }
    else if (game_state.getState() == state_name::penalty)
    {
      ai::Team team_penalty = game_state.penaltyTeam();
      if (team_penalty == get_team())
      {
        DEBUG("PENALTY ALLY");
        start_penalty_ally();
      }
      else
      {
        DEBUG("PENALTY OPPONENT");
        start_penalty_opponent();
      }
    }
  }
  else
  {
    // clear_strategy_assignement();
    if (game_state.getState() == state_name::running)
    {
      continue_running();
    }
    else if (game_state.getState() == state_name::stop)
    {
      // DEBUG("STOP continue");
      continue_stop();
    }
    else if (game_state.getState() == state_name::halt)
    {
      // DEBUG("HALT continue");
      continue_halt();
    }
    else if (game_state.getState() == state_name::free_kick)
    {
      free_kick_type_id type_free_kick = game_state.typeOfTheFreeKick();
      ai::Team team_free_kick = game_state.freeKickTeam();
      if (type_free_kick == DIRECT and team_free_kick == get_team())
      {
        // DEBUG("DIRECT KICK ALLY continue");
        continue_direct_kick_ally();
      }
      else if (type_free_kick == DIRECT and team_free_kick != get_team())
      {
        // DEBUG("DIRECT KICK OPPONENT continue");
        continue_direct_kick_opponent();
      }
      else if (type_free_kick == INDIRECT and team_free_kick == get_team())
      {
        // DEBUG("INDIRECT KICK ALLY continue");
        continue_indirect_kick_ally();
      }
      else if (type_free_kick == INDIRECT and team_free_kick != get_team())
      {
        // DEBUG("INDIRECT KICK OPPONENT continue");
        continue_indirect_kick_opponent();
      }
    }
    else if (game_state.getState() == state_name::prepare_kickoff)
    {
      ai::Team team_prepare_kickoff = game_state.kickoffTeam();
      if (team_prepare_kickoff == get_team())
      {
        // DEBUG("PREPARE KICKOFF ALLY continue");
        continue_prepare_kickoff_ally();
      }
      else
      {
        // DEBUG("PREPARE KICKOFF OPPONENT continue");
        continue_prepare_kickoff_opponent();
      }
    }
    else if (game_state.getState() == state_name::kickoff)
    {
      ai::Team team_kickoff = game_state.kickoffTeam();
      if (team_kickoff == get_team())
      {
        // DEBUG("KICKOFF ALLY continue");
        continue_kickoff_ally();
      }
      else
      {
        // DEBUG("KICKOFF OPPONENT continue");
        continue_kickoff_opponent();
      }
    }
    else if (game_state.getState() == state_name::penalty)
    {
      ai::Team team_penalty = game_state.penaltyTeam();
      if (team_penalty == get_team())
      {
        // DEBUG("PENALTY ALLY continue");
        continue_penalty_ally();
      }
      else
      {
        // DEBUG("PENALTY OPPONENT continue");
        continue_penalty_opponent();
      }
    }
  }
}

void ManagerWithGameState::update(double time)
{
  // update_strategies(time);
  update_current_strategies(time);
  analyse_data(time);
  choose_a_strategy(time);
}

ManagerWithGameState::~ManagerWithGameState()
{
}

};  // namespace Manager
};  // namespace rhoban_ssl
