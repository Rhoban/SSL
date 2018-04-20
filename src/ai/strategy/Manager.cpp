#include "Manager.h"

// The different strategies
#include "halt.h"
#include "tare_and_synchronize.h"
#include "sandbox.h"

namespace RhobanSSL {
namespace Strategy {

Manager::Manager(Ai::AiData & game_state): game_state(game_state) {
    current_strategy_name = "";

    register_strategy( Halt::name, std::shared_ptr<Strategy>( new Halt() ) );
    register_strategy(
        Tare_and_synchronize::name,
        std::shared_ptr<Strategy>( new Tare_and_synchronize() )
    );
    register_strategy(
        Sandbox::name,
        std::shared_ptr<Strategy>( new Sandbox(game_state) )
    );
    sandbox = false;
    start = -1.0;
    assign_strategy( Halt::name, 0.0 ); // TODO TIME !
}

void Manager::register_strategy(
    const std::string& strategy_name, std::shared_ptr<Strategy>  strategy
){
    assert( strategies.find(strategy_name) == strategies.end() );
    strategies[strategy_name] = strategy;
}

void Manager::assign_strategy( const std::string & strategy_name, double time ){
    assert( strategies.find(strategy_name) != strategies.end() );
    if( current_strategy_name != ""){
        current_strategy().stop(time);
    }
    current_strategy_name = strategy_name;
    current_strategy().start(time);
}

const std::string & Manager::strategy_name() const{
    return current_strategy_name;
}

void Manager::analyse_data(double time){
}
void Manager::choose_a_strategy(double time){
    if(start==-1.0){ 
        assign_strategy( Tare_and_synchronize::name, time );
        start = time;
        return;
    }

    if( 
        ! get_strategy<Tare_and_synchronize>().is_tared_and_synchronized()
    ){
        return;
    }

    if( ! sandbox ){
        assign_strategy( Sandbox::name, time );
        sandbox = true;
    }
}

void Manager::update_strategies(double time){
    for( std::pair< std::string, std::shared_ptr<Strategy> > elem : strategies ){
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
    current_strategy().assign_behavior_to_robots(robot_behaviors, time, dt);
}

Strategy & Manager::current_strategy(){
    return *strategies.at(current_strategy_name);
}


};
};
