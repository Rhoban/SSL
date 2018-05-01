#ifndef __MANAGER__MANAGER__H__
#define __MANAGER__MANAGER__H__

#include <strategy/Strategy.h>
#include <robot_behavior/robot_behavior.h>
#include <map>
#include <memory>
#include <vector>
#include <AiData.h>

namespace RhobanSSL {
namespace Manager {

class Manager {
    private:

    bool blueIsNotSet;
    bool blueTeamOnPositiveHalf;
    Ai::Team team;
    int goalie_id;
    std::vector<int> team_ids;

    std::string current_strategy_name;
    std::map< std::string, std::shared_ptr<Strategy::Strategy>> strategies;

    protected:
    Ai::AiData & game_state;
    
    public:
    Manager( Ai::AiData& game_state );

    void set_team( Ai::Team team );
    Ai::Team get_team() const;
    void declare_goalie_id(
        int goalie_id
    );
    void declare_team_ids(
        const std::vector<int> & team_ids
    );
    const std::vector<int> & get_team_ids() const;
    int get_goalie_id() const;   

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
 
    virtual void update(double time) = 0;

    virtual void update_strategies(double time);
    virtual void update_current_strategy(double time);

    virtual void assign_behavior_to_robots(
        std::map<
            int, 
            std::shared_ptr<RobotBehavior>
        > & robot_behaviors, double time, double dt
    );

    void change_team_point_of_view( bool blue_have_it_s_goal_on_positive_x_axis );

    virtual ~Manager();
};

};
};

#endif
