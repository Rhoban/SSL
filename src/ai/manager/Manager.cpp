#include "Manager.h"

#include <debug.h>
#include <strategy/halt.h>

namespace RhobanSSL {
namespace Manager {

#define MANAGER__REMOVE_ROBOTS "manager__remove_robots"

int Manager::get_goalie_opponent_id() const {
    return goalie_opponent_id;
}

void Manager::declare_goalie_opponent_id(
    int goalie_opponent_id
){
    this->goalie_opponent_id = goalie_opponent_id;
}
void Manager::declare_goalie_id(
    int goalie_id
){
    this->goalie_id = goalie_id;
}
int Manager::get_goalie_id() const {
    return goalie_id;
}
void Manager::declare_team_ids(
    const std::vector<int> & team_ids
){
    this->team_ids = team_ids;
}

Ai::Team Manager::get_team() const{
     return game_state.team_color;
}
const std::string & Manager::get_team_name() const{
     return game_state.team_name;
}

const std::vector<int> & Manager::get_team_ids() const {
    return team_ids;
}

void Manager::register_strategy(
    const std::string& strategy_name,
    std::shared_ptr<Strategy::Strategy>  strategy
){
    assert( strategies.find(strategy_name) == strategies.end() );
    strategies[strategy_name] = strategy;
}

void Manager::clear_strategy_assignement(){
    for( const std::string & name : current_strategy_names ){
        get_strategy(name).stop(time());
    }
    current_strategy_names.clear();
    assign_strategy(
        MANAGER__REMOVE_ROBOTS, time(), get_invalid_team_ids() 
    );
}



void Manager::assign_strategy(
    const std::string & strategy_name, 
    double time, const std::vector<int> & robot_ids
){
    assert( strategies.find(strategy_name) != strategies.end() );

    current_strategy_names.push_front( strategy_name );
    Strategy::Strategy & strategy = get_strategy( strategy_name ); 
    
    strategy.set_goalie( goalie_id );
    strategy.set_goalie_opponent( goalie_opponent_id );
    strategy.set_robot_affectation( robot_ids );
    strategy.start(time);
}

Strategy::Strategy & Manager::get_strategy( const std::string & strategy_name ) {
    assert( strategies.find(strategy_name) != strategies.end() );
    return *(strategies.at(strategy_name)); 
}

const Strategy::Strategy & Manager::get_strategy( const std::string & strategy_name ) const {
    assert( strategies.find(strategy_name) != strategies.end() );
    return *(strategies.at(strategy_name)); 
}

const std::list<std::string> & Manager::get_current_strategy_names() const{
    return current_strategy_names;
}

void Manager::update_strategies(double time){
    for(
        std::pair< 
            std::string, std::shared_ptr<Strategy::Strategy> 
        > elem : strategies
    ){
        elem.second->update(time);
    }
}

void Manager::update_current_strategies(double time){
    for( const std::string & name : current_strategy_names ){
        get_strategy(name).update( time );
    }
}

void Manager::assign_behavior_to_robots(
    std::map<
        int, 
        std::shared_ptr<Robot_behavior::RobotBehavior>
    > & robot_behaviors,
    double time, double dt
){
    for( const std::string & name : current_strategy_names ){
        get_strategy(name).assign_behavior_to_robots(
            [&](int id, std::shared_ptr<Robot_behavior::RobotBehavior> behavior){
                 return  
               robot_behaviors[id] = behavior; 
            }, time, dt
        );
    }
}

void Manager::change_ally_and_opponent_goalie_id( int blue_goalie_id, int yellow_goalie_id){
    declare_goalie_id(
        (get_team() == Ai::Team::Yellow)? yellow_goalie_id : blue_goalie_id
    );
    declare_goalie_opponent_id(
        (get_team() == Ai::Team::Yellow)? blue_goalie_id : yellow_goalie_id
    );
}



void Manager::change_team_and_point_of_view( Ai::Team team, bool blue_have_it_s_goal_on_positive_x_axis ){
    
    if( team != Ai::Unknown and get_team() != team ){
        assert( team == Ai::Blue or team == Ai::Yellow );
        game_state.change_team_color( team );
        blueIsNotSet = true;
    }
    // We change the point of view of the team
    if( 
        blueIsNotSet
        or
        blueTeamOnPositiveHalf != blue_have_it_s_goal_on_positive_x_axis
    ){
        blueIsNotSet = false;
        blueTeamOnPositiveHalf = blue_have_it_s_goal_on_positive_x_axis;
        if(
            (
                get_team() == Ai::Blue
                and 
                blue_have_it_s_goal_on_positive_x_axis
            )or(
                get_team() == Ai::Yellow
                and 
                ! blue_have_it_s_goal_on_positive_x_axis
            )
        ){
            game_state.change_frame_for_all_objects(
                rhoban_geometry::Point(0.0,0.0),
                Vector2d(-1.0, 0.0), Vector2d(0.0, -1.0)
            );
        }else{
            game_state.change_frame_for_all_objects(
                rhoban_geometry::Point(0.0,0.0),
                Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)
            );
        }
    }

}

Manager::Manager( Ai::AiData& game_state ):
    blueIsNotSet(true),
    goalie_id(-1),
    goalie_opponent_id(-1),
    game_state(game_state)
{
    register_strategy(
        MANAGER__REMOVE_ROBOTS, std::shared_ptr<Strategy::Strategy>(
            new Strategy::Halt(game_state) 
        )
    );
}

int Manager::time() const {
    return game_state.time;
}

int Manager::dt() const {
    return game_state.dt;
}

void Manager::affect_invalid_robots_to_invalid_robots_strategy(){
    Strategy::Strategy & strategy = get_strategy( MANAGER__REMOVE_ROBOTS );
    strategy.stop(time());
    strategy.set_robot_affectation( get_invalid_team_ids() );
    strategy.start(time());
}

void Manager::remove_invalid_robots(){
    detect_invalid_robots();
    affect_invalid_robots_to_invalid_robots_strategy();
}

void Manager::detect_invalid_robots(){
    int nb_valid = 0;
    int n_robots = team_ids.size();
    for(int i=0; i<n_robots; i++ ){
        if( game_state.robot_is_valid( team_ids[i] ) ){
            nb_valid ++ ;
        }
    }
    valid_team_ids.clear();
    invalid_team_ids.clear();
    for(int i=0; i<n_robots; i++ ){
        int id = team_ids[i];
        if( game_state.robot_is_valid( id ) ){
            valid_team_ids.push_back( id );
        }else{
            invalid_team_ids.push_back( id );
        }
    }
}

const std::vector<int> & Manager::get_valid_team_ids() const {
    return valid_team_ids;
}
const std::vector<int> & Manager::get_invalid_team_ids() const {
    return invalid_team_ids;
}
Manager::~Manager(){ }

};
};
