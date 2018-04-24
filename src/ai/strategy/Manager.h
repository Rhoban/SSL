#ifndef __STRATEGY__MANAGER__H__
#define __STRATEGY__MANAGER__H__

#include "Strategy.h"
#include <robot_behavior/robot_behavior.h>
#include <referee/Referee.h>
#include <map>
#include <memory>
#include <AiData.h>
#include <list>

namespace RhobanSSL {
namespace Strategy {

class Manager {
    private:
    std::list<int> team_ids;

    std::string current_strategy_name;
    std::map< std::string, std::shared_ptr<Strategy>> strategies;
    Ai::AiData& game_state;
    const Referee & referee;
    
    double start;
    bool sandbox;
    public:

    void declare_team_ids(
        const std::list<int> & team_ids
    );
    const std::list<int> & get_team_ids() const;
    
    Manager(
        Ai::AiData & game_state,
        const Referee & referee
    );

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
   
    void assign_strategy(
        const std::string & strategy_name, double time,
        const std::list<int> & robot_ids
    );
 
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
