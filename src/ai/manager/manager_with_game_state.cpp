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
  if(game_state.state_is_newer(last_change_stamp)){
    last_change_stamp = game_state.get_change_stamp();
    if(game_state.get_state() == state_name::running){
      start_running();
    }

  }else{
    if(game_state.get_state() == state_name::running){
      // continue_running();
    }
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
