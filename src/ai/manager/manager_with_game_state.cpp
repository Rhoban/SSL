#include "manager_with_game_state.h"


namespace RhobanSSL
{
namespace Manager
{

ManagerWithGameState::ManagerWithGameState(Ai::AiData &ai_data,
    const GameState &game_state) : Manager(ai_data),
                              game_state(game_state),
                              last_change_stamp(0)
{
}

void ManagerWithGameState::analyse_data(double time)
{
    // We change the point of view of the team
    change_team_and_point_of_view(
        game_state.get_team_color(get_team_name()),
        game_state.blue_have_it_s_goal_on_positive_x_axis());
    change_ally_and_opponent_goalie_id(
        game_state.blue_goalie_id(),
        game_state.yellow_goalie_id());
}

void ManagerWithGameState::choose_a_strategy(double time)
{
  if (game_state.state_is_newer(last_change_stamp)){
    clear_strategy_assignement();
    last_change_stamp = game_state.get_change_stamp();
    if (game_state.get_state() == state_name::running){
      DEBUG("RUNNING");
      start_running();
    }
    else if (game_state.get_state() == state_name::stop){
      DEBUG("STOP");
      start_stop();
    }
    else if (game_state.get_state() == state_name::halt){
      DEBUG("HALT");
      start_halt();
    }
    else if (game_state.get_state() == state_name::free_kick){
      free_kick_type_id type_free_kick = game_state.type_of_the_free_kick();
      Ai::Team team_free_kick = game_state.free_kick_team();
      if (type_free_kick == DIRECT and team_free_kick == get_team()){
        DEBUG("DIRECT KICK ALLY");
        start_direct_kick_ally();
      }
      else if (type_free_kick == DIRECT and team_free_kick != get_team()){
        DEBUG("DIRECT KICK OPPONENT");
        start_direct_kick_opponent();
      }
      else if (type_free_kick == INDIRECT and team_free_kick == get_team()){
        DEBUG("INDIRECT KICK ALLY");
        start_indirect_kick_ally();
      }
      else if (type_free_kick == INDIRECT and team_free_kick != get_team()){
        DEBUG("INDIRECT KICK OPPONENT");
        start_indirect_kick_opponent();
      }
    }
    else if (game_state.get_state() == state_name::prepare_kickoff){
      Ai::Team team_prepare_kickoff = game_state.kickoff_team();
      if(team_prepare_kickoff == get_team()){
        DEBUG("PREPARE KICKOFF ALLY");
        start_prepare_kickoff_ally();
      }
      else{
        DEBUG("PREPARE KICKOFF OPPONENT");
        start_prepare_kickoff_opponent();
      }
    }
    else if (game_state.get_state() == state_name::kickoff){
      Ai::Team team_kickoff = game_state.kickoff_team();
      if(team_kickoff == get_team()){
        DEBUG("KICKOFF ALLY");
        start_kickoff_ally();
      }
      else{
        DEBUG("KICKOFF OPPONENT");
        start_kickoff_opponent();
      }
    }
    else if (game_state.get_state() == state_name::penalty){
      Ai::Team team_penalty = game_state.penalty_team();
      if(team_penalty == get_team()){
        DEBUG("PENALTY ALLY");
        start_penalty_ally();
      }
      else{
        DEBUG("PENALTY OPPONENT");
        start_penalty_opponent();
      }
    }
    
    

  }else{
    // clear_strategy_assignement();
    if (game_state.get_state() == state_name::running){
      continue_running();
    }
    // else if (game_state.get_state() == state_name::stop){
    //   DEBUG("STOP continue");
    //   continue_stop();
    // }
    // else if (game_state.get_state() == state_name::halt){
    //   DEBUG("HALT continue");
    //   continue_halt();
    // }
    // else if (game_state.get_state() == state_name::free_kick){
    //   free_kick_type_id type_free_kick = game_state.type_of_the_free_kick();
    //   Ai::Team team_free_kick = game_state.free_kick_team();
    //   if (type_free_kick == DIRECT and team_free_kick == get_team()){
    //     DEBUG("DIRECT KICK ALLY continue");
    //     continue_direct_kick_ally();
    //   }
    //   else if (type_free_kick == DIRECT and team_free_kick != get_team()){
    //     DEBUG("DIRECT KICK OPPONENT continue");
    //     continue_direct_kick_opponent();
    //   }
    //   else if (type_free_kick == INDIRECT and team_free_kick == get_team()){
    //     DEBUG("INDIRECT KICK ALLY continue");
    //     continue_indirect_kick_ally();
    //   }
    //   else if (type_free_kick == INDIRECT and team_free_kick != get_team()){
    //     DEBUG("INDIRECT KICK OPPONENT continue");
    //     continue_indirect_kick_opponent();
    //   }
    // }
    // else if (game_state.get_state() == state_name::prepare_kickoff){
    //   Ai::Team team_prepare_kickoff = game_state.kickoff_team();
    //   if(team_prepare_kickoff == get_team()){
    //     DEBUG("PREPARE KICKOFF ALLY continue");
    //     continue_prepare_kickoff_ally();
    //   }
    //   else{
    //     DEBUG("PREPARE KICKOFF OPPONENT continue");
    //     continue_prepare_kickoff_opponent();
    //   }
    // }
    // else if (game_state.get_state() == state_name::kickoff){
    //   Ai::Team team_kickoff = game_state.kickoff_team();
    //   if(team_kickoff == get_team()){
    //     DEBUG("KICKOFF ALLY continue");
    //     continue_kickoff_ally();
    //   }
    //   else{
    //     DEBUG("KICKOFF OPPONENT continue");
    //     continue_kickoff_opponent();
    //   }
    // }
    // else if (game_state.get_state() == state_name::penalty){
    //   Ai::Team team_penalty = game_state.penalty_team();
    //   if(team_penalty == get_team()){
    //     DEBUG("PENALTY ALLY continue");
    //     continue_penalty_ally();
    //   }
    //   else{
    //     DEBUG("PENALTY OPPONENT continue");
    //     continue_penalty_opponent();
    //   }
  }

}

void ManagerWithGameState::update(double time)
{
    //update_strategies(time);
    update_current_strategies(time);
    analyse_data(time);
    choose_a_strategy(time);
}

ManagerWithGameState::~ManagerWithGameState() {}

}; // namespace Manager
}; // namespace RhobanSSL
