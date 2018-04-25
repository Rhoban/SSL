#include "Manager.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/sandbox.h>

namespace RhobanSSL {
namespace Manager {



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
const std::vector<int> & Manager::get_team_ids() const {
    return team_ids;
}

Manager::Manager(
    Ai::AiData & game_state,
    const Referee & referee
):
    game_state(game_state),
    referee(referee)
{
    current_strategy_name = "";

    register_strategy(
        Strategy::Halt::name, std::shared_ptr<Strategy::Strategy>(
            new Strategy::Halt() 
        )
    );
    register_strategy(
        Strategy::Tare_and_synchronize::name,
        std::shared_ptr<Strategy::Strategy>( 
            new Strategy::Tare_and_synchronize()
        )
    );
    register_strategy(
        Strategy::Sandbox::name,
        std::shared_ptr<Strategy::Strategy>(
            new Strategy::Sandbox(game_state)
        )
    );
    sandbox = false;
    start = -1.0;
    assign_strategy( Strategy::Halt::name, 0.0, team_ids ); // TODO TIME !
}

void Manager::register_strategy(
    const std::string& strategy_name,
    std::shared_ptr<Strategy::Strategy>  strategy
){
    assert( strategies.find(strategy_name) == strategies.end() );
    strategies[strategy_name] = strategy;
}

void Manager::assign_strategy(
    const std::string & strategy_name, 
    double time, const std::vector<int> & robot_ids
){
    assert( strategies.find(strategy_name) != strategies.end() );
    if( current_strategy_name != ""){
        current_strategy().stop(time);
    }
    current_strategy_name = strategy_name;
    
    current_strategy().set_goalie( goalie_id );
    current_strategy().set_robot_affectation( robot_ids );
    current_strategy().start(time);
}

const std::string & Manager::strategy_name() const{
    return current_strategy_name;
}

void Manager::analyse_data(double time){
}
void Manager::choose_a_strategy(double time){
    if(start==-1.0){ 
        assign_strategy(
            Strategy::Tare_and_synchronize::name, time,
            team_ids
        );
        start = time;
        return;
    }

    if( 
        ! get_strategy<Strategy::Tare_and_synchronize>().is_tared_and_synchronized()
    ){
        return;
    }

    if( ! sandbox ){
        assign_strategy(
            Strategy::Sandbox::name, time,
            team_ids
        );
        sandbox = true;
    }
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

void Manager::update_current_strategy(double time){
    current_strategy().update( time );
}

void Manager::update(double time){
    //update_strategies(time);
    update_current_strategy(time);
    analyse_data(time);
    choose_a_strategy(time);
}

void Manager::assign_behavior_to_robots(
    std::map<
        int, 
        std::shared_ptr<RobotBehavior>
    > & robot_behaviors,
    double time, double dt
){
    current_strategy().assign_behavior_to_robots(
        [&](int id, std::shared_ptr<RobotBehavior> behavior){
             return  
           robot_behaviors[id] = behavior; 
        }, time, dt
    );
}

Strategy::Strategy & Manager::current_strategy(){
    return *strategies.at(current_strategy_name);
}


};
};
