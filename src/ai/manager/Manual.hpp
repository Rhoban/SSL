#ifndef __MANAGER__MANUAL__H__
#define __MANAGER__MANUAL__H__

#include "Manager.h"

namespace RhobanSSL {
namespace Manager {

class Manual {
    private:
    int goalie_id;
    std::vector<int> team_ids;

    std::string current_strategy_name;
    std::map< std::string, std::shared_ptr<Strategy::Strategy>> strategies;
    Ai::AiData& game_state;
    const Referee & referee;
    
    double start;
    bool sandbox;
    public:

    void declare_goalie_id(
        int goalie_id
    );
    void declare_team_ids(
        const std::vector<int> & team_ids
    );
    const std::vector<int> & get_team_ids() const;
    int get_goalie_id() const;   
 
    Manual(
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

    Strategy::Strategy & current_strategy();
    const std::string & strategy_name() const;

    void register_strategy(
        const std::string& strategy_name,
        std::shared_ptr<Strategy::Strategy> strategy
    );
   
    void assign_strategy(
        const std::string & strategy_name, double time,
        const std::vector<int> & robot_ids
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
