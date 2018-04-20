#ifndef __STRATEGY__MANAGER__H__
#define __STRATEGY__MANAGER__H__

#include "Strategy.h"
#include <robot_behavior/robot_behavior.h>
#include <map>
#include <memory>
#include <AiData.h>

namespace RhobanSSL {
namespace Strategy {

class Manager {
    private:
    std::string current_strategy_name;
    std::map< std::string, std::shared_ptr<Strategy>> strategies;
    Ai::AiData& game_state;
    
    double start;
    bool sandbox;
    public:
    
    Manager(Ai::AiData & game_state);

    template <typename STRATEGY>
    STRATEGY & get_strategy( const std::string & name ){
        return static_cast<STRATEGY&>( *strategies.at( name ) );
    };

    template <typename STRATEGY>
    STRATEGY & get_strategy(){
        return get_strategy<STRATEGY>( STRATEGY::name );
    };

    Strategy & current_strategy();
    const std::string & strategy_name() const;

    void register_strategy(
        const std::string& strategy_name, std::shared_ptr<Strategy> strategy
    );
   
    void assign_strategy( const std::string & strategy_name, double time );
 
    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);
    void update_strategies(double time);
    void update_current_strategy(double time);
    void assign_behavior_to_robots(
        std::map<
            int, 
            std::shared_ptr<RobotBehavior>
        > & robot_behaviors, double time, double dt
    );
};

};
};

#endif
