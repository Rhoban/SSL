#include "manager_with_game_state.h"

namespace rhoban_ssl
{
namespace manager
{
ManagerWithGameState::ManagerWithGameState() : Manager(), last_change_stamp_(0)
{
}

void ManagerWithGameState::analyseData(double time)
{
}

void ManagerWithGameState::chooseAStrategy(double time)
{
  GameState& game_state = GlobalDataSingleThread::singleton_.referee_.game_state;
  if (game_state.stateIsNewer(last_change_stamp_))
  {
    clearStrategyAssignement();
    last_change_stamp_ = game_state.getChangeStamp();
    if (game_state.getState() == state_name::running)
    {
      DEBUG("RUNNING");
      startRunning();
    }
    else if (game_state.getState() == state_name::stop)
    {
      DEBUG("STOP");
      startStop();
    }
    else if (game_state.getState() == state_name::halt)
    {
      DEBUG("HALT");
      startHalt();
    }
    else if (game_state.getState() == state_name::free_kick)
    {
      free_kick_type_id type_free_kick = game_state.typeOfTheFreeKick();
      Team team_free_kick = game_state.freeKickTeam();
      if (type_free_kick == DIRECT and team_free_kick == Ally)
      {
        DEBUG("DIRECT KICK ALLY");
        startDirectKickAlly();
      }
      else if (type_free_kick == DIRECT and team_free_kick == Opponent)
      {
        DEBUG("DIRECT KICK OPPONENT");
        startDirectKickOpponent();
      }
      else if (type_free_kick == INDIRECT and team_free_kick == Ally)
      {
        DEBUG("INDIRECT KICK ALLY");
        startIndirectKickAlly();
      }
      else if (type_free_kick == INDIRECT and team_free_kick == Opponent)
      {
        DEBUG("INDIRECT KICK OPPONENT");
        startIndirectKickOpponent();
      }
    }
    else if (game_state.getState() == state_name::prepare_kickoff)
    {
      Team team_prepare_kickoff = game_state.kickoffTeam();
      if (team_prepare_kickoff == Ally)
      {
        DEBUG("PREPARE KICKOFF ALLY");
        startPrepareKickoffAlly();
      }
      else
      {
        DEBUG("PREPARE KICKOFF OPPONENT");
        startPrepareKickoffOpponent();
      }
    }
    else if (game_state.getState() == state_name::kickoff)
    {
      Team team_kickoff = game_state.kickoffTeam();
      if (team_kickoff == Ally)
      {
        DEBUG("KICKOFF ALLY");
        startKickoffAlly();
      }
      else
      {
        DEBUG("KICKOFF OPPONENT");
        startKickoffOpponent();
      }
    }
    else if (game_state.getState() == state_name::penalty)
    {
      Team team_penalty = game_state.penaltyTeam();
      if (team_penalty == Ally)
      {
        DEBUG("PENALTY ALLY");
        startPenaltyAlly();
      }
      else
      {
        DEBUG("PENALTY OPPONENT");
        startPenaltyOpponent();
      }
    }
  }
  else
  {
    // clear_strategy_assignement();
    if (game_state.getState() == state_name::running)
    {
      continueRunning();
    }
    else if (game_state.getState() == state_name::stop)
    {
      // DEBUG("STOP continue");
      continueStop();
    }
    else if (game_state.getState() == state_name::halt)
    {
      // DEBUG("HALT continue");
      continueHalt();
    }
    else if (game_state.getState() == state_name::free_kick)
    {
      free_kick_type_id type_free_kick = game_state.typeOfTheFreeKick();
      Team team_free_kick = game_state.freeKickTeam();
      if (type_free_kick == DIRECT and team_free_kick == Ally)
      {
        // DEBUG("DIRECT KICK ALLY continue");
        continueDirectKickAlly();
      }
      else if (type_free_kick == DIRECT and team_free_kick == Opponent)
      {
        // DEBUG("DIRECT KICK OPPONENT continue");
        continueDirectKickOpponent();
      }
      else if (type_free_kick == INDIRECT and team_free_kick == Ally)
      {
        // DEBUG("INDIRECT KICK ALLY continue");
        continueIndirectKickAlly();
      }
      else if (type_free_kick == INDIRECT and team_free_kick == Opponent)
      {
        // DEBUG("INDIRECT KICK OPPONENT continue");
        continueIndirectKickOpponent();
      }
    }
    else if (game_state.getState() == state_name::prepare_kickoff)
    {
      Team team_prepare_kickoff = game_state.kickoffTeam();
      if (team_prepare_kickoff == Ally)
      {
        // DEBUG("PREPARE KICKOFF ALLY continue");
        continuePrepareKickoffAlly();
      }
      else
      {
        // DEBUG("PREPARE KICKOFF OPPONENT continue");
        continuePrepareKickoffOpponent();
      }
    }
    else if (game_state.getState() == state_name::kickoff)
    {
      Team team_kickoff = game_state.kickoffTeam();
      if (team_kickoff == Ally)
      {
        // DEBUG("KICKOFF ALLY continue");
        continueKickoffAlly();
      }
      else
      {
        // DEBUG("KICKOFF OPPONENT continue");
        continueKickoffOpponent();
      }
    }
    else if (game_state.getState() == state_name::penalty)
    {
      Team team_penalty = game_state.penaltyTeam();
      if (team_penalty == Ally)
      {
        // DEBUG("PENALTY ALLY continue");
        continuePenaltyAlly();
      }
      else
      {
        // DEBUG("PENALTY OPPONENT continue");
        continuePenaltyOpponent();
      }
    }
  }
}

void ManagerWithGameState::update(double time)
{
  // update_strategies(time);
  updateCurrentStrategies(time);
  analyseData(time);
  chooseAStrategy(time);
}

ManagerWithGameState::~ManagerWithGameState()
{
}

};  // namespace manager
};  // namespace rhoban_ssl
