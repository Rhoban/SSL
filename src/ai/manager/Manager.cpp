#include "Manager.h"

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


Manager::~Manager(){ }

};
};
