#ifndef __MANAGER__MANAGER__H__
#define __MANAGER__MANAGER__H__

#include <strategy/Strategy.h>
#include <robot_behavior/robot_behavior.h>
#include <map>
#include <memory>
#include <vector>
#include <AiData.h>
#include <strategy/placer.h>

namespace RhobanSSL {
namespace Manager {

class Manager {
    public:

        static constexpr const char* MANAGER__REMOVE_ROBOTS = "manager__remove_robots";
        static constexpr const char* MANAGER__PLACER = "manager__placer";

    private:

    bool blueIsNotSet;
    bool blueTeamOnPositiveHalf;
    int goalie_id;
    int goalie_opponent_id;
    std::vector<int> team_ids;
    std::vector<int> valid_team_ids;
    std::vector<int> valid_player_ids;
    std::vector<int> invalid_team_ids;

    std::list<std::string> current_strategy_names;
    std::map< std::string, std::shared_ptr<Strategy::Strategy>> strategies;

    void affect_invalid_robots_to_invalid_robots_strategy();
    void detect_invalid_robots();

    protected:
    Ai::AiData & ai_data;

    public:
    int time() const ;
    int dt() const ;

    std::vector<std::string> get_available_strategies();

    Manager( Ai::AiData& ai_data );

    Ai::Team get_team() const;
    const std::string & get_team_name() const;
    void declare_goalie_id( int goalie_id );
    void declare_goalie_opponent_id( int goalie_opponent_id );
    void declare_team_ids( const std::vector<int> & team_ids );
    const std::vector<int> & get_team_ids() const;
    const std::vector<int> & get_valid_team_ids() const;
    const std::vector<int> & get_valid_player_ids() const;
    const std::vector<int> & get_invalid_team_ids() const;
    // return the goalie id. If id<0 then no goalie is declared.
    int get_goalie_id() const;
    // return the opponent goalie id. If id<0 then no opponent goalie is declared.
    int get_goalie_opponent_id() const;

    template <typename STRATEGY>
    STRATEGY & get_strategy_( const std::string & name ){
        return static_cast<STRATEGY&>( *strategies.at( name ) );
    };

    template <typename STRATEGY>
    STRATEGY & get_strategy_(){
        return get_strategy_<STRATEGY>( STRATEGY::name );
    };

    Strategy::Strategy & get_strategy( const std::string & strategy_name );
    const Strategy::Strategy & get_strategy( const std::string & strategy_name ) const;
    const std::list<std::string> & get_current_strategy_names() const;

    void register_strategy(
        const std::string& strategy_name,
        std::shared_ptr<Strategy::Strategy> strategy
    );

    void clear_strategy_assignement();

    void assign_strategy(
        const std::string & strategy_name, double time,
        const std::vector<int> & robot_ids, bool assign_goalie=false
    );

    virtual void update(double time) = 0;

    virtual void update_strategies(double time);
    virtual void update_current_strategies(double time);

    virtual void assign_behavior_to_robots(
        std::map<
            int,
            std::shared_ptr<Robot_behavior::RobotBehavior>
        > & robot_behaviors, double time, double dt
    );

    void change_ally_and_opponent_goalie_id( int blue_goalie_id, int yellow_goalie_id);

    void change_team_and_point_of_view( Ai::Team team, bool blue_have_it_s_goal_on_positive_x_axis );

    void remove_invalid_robots();

    virtual ~Manager();

    private:


    void aggregate_all_starting_position_of_all_strategies();
        std::list<
            std::pair<rhoban_geometry::Point,ContinuousAngle>
        > starting_positions;
        std::list<
            std::pair<std::string, int>
        > repartitions_of_starting_positions;
        bool goal_has_to_be_placed;
        rhoban_geometry::Point goalie_linear_position;
        ContinuousAngle goalie_angular_position;

    void determine_the_robot_needs_for_the_strategies();
        std::map<std::string, std::vector<int>> robot_affectations_by_strategy;
        std::map<std::string, int> number_of_extra_robot_by_strategy;
        unsigned int nb_of_extra_robots;
        unsigned int nb_of_extra_robots_non_affected;
    void compute_robot_affectations_to_strategies();
        unsigned int extra_robots = 0;

    void sort_robot_ordered_by_the_distance_with_starting_position();
        std::vector<int> robot_affectations;
        std::vector<
            std::pair<rhoban_geometry::Point, ContinuousAngle>
        > robot_consigns;
        std::pair<rhoban_geometry::Point, ContinuousAngle> goalie_consigns;


    void declare_robot_positions_in_the_placer();
    
    protected:
    void declare_next_strategies(const std::list<std::string> & next_strategies);
    private:
        std::list<std::string> next_strategies;

    public:
    void place_all_the_robots(
        double time, const std::list<std::string> & next_strategies
    );
    const std::vector<int> & get_robot_affectations( const std::string & strategy_name ) const;

    // Return the ally robot whose number is 
    // `̀robot_number'.
    Ai::Robot& robot( int robot_number) const;

};


};
};

#endif
