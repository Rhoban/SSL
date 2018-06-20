/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __MANAGER__MANAGER__H__
#define __MANAGER__MANAGER__H__

#include <game_informations.h>
#include <strategy/Strategy.h>
#include <robot_behavior/robot_behavior.h>
#include <map>
#include <memory>
#include <vector>
#include <AiData.h>
#include <strategy/placer.h>
#include <annotations/Annotations.h>

namespace RhobanSSL {
namespace Manager {

class Manager : public GameInformations {
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
    const std::string & get_next_strategy_with_goalie() const;
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
    void declare_and_assign_next_strategies(const std::list<std::string> & future_strats);

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

    void determine_the_robot_needs_for_the_strategies(
        const std::list<std::string> & next_strategies
    );
        unsigned int nb_of_robots_to_be_affected;
        unsigned int nb_of_extra_robots_non_affected;
        unsigned int minimal_nb_of_robots_to_be_affected;
        unsigned int nb_of_extra_robots;
        std::string strategy_with_arbitrary_number_of_robot;
        //init of robot_affectations_by_strategy;
        bool goal_has_to_be_placed;
        std::string strategy_with_goal;

    void aggregate_all_starting_position_of_all_strategies(
        const std::list<std::string> & next_strategies
    );
        std::list<
            std::pair<rhoban_geometry::Point,ContinuousAngle>
        > starting_positions;
        std::list<
            std::pair<std::string, int>
        > repartitions_of_starting_positions_in_the_list;
        rhoban_geometry::Point goalie_linear_position;
        ContinuousAngle goalie_angular_position;

    void sort_robot_ordered_by_the_distance_with_starting_position();
        std::vector<int> robot_affectations; 
            // this list is a special orrder of get_valid_player_ids(). 
            // ths starting_posiitons.size() fisrt robots of robot_affectation
            // correspond a good affectation with respect to the starting position 
            // of startings position;
        std::vector<
            std::pair<rhoban_geometry::Point, ContinuousAngle>
        > robot_consigns;  
            // this list contains the starting position of get_valid_player_ids().
            // the starting_posiitons.size() first position are equals to the position
            // pf starting_position.
            // the other are some default placement.

    void compute_robot_affectations_to_strategies();
        std::map<std::string, std::vector<int>> robot_affectations_by_strategy;
        unsigned int extra_robots = 0;




/*

    void determine_the_robot_needs_for_the_strategies();
        std::map<std::string, std::vector<int>> robot_affectations_by_strategy;
        std::map<std::string, int> number_of_extra_robot_by_strategy;
        unsigned int nb_of_extra_robots;
        unsigned int nb_of_extra_robots_non_affected;
    void compute_robot_affectations_to_strategies();
        unsigned int extra_robots = 0;

*/

    void declare_robot_positions_in_the_placer();
    
    protected:
    void declare_next_strategies(const std::list<std::string> & next_strategies);

    public:
    void set_ball_avoidance_for_all_robots( bool value );

    void place_all_the_robots(
        double time, const std::list<std::string> & next_strategies
    );
    const std::vector<int> & get_robot_affectations( const std::string & strategy_name ) const;

    //
    // This function is used to draw annotations in the viewer.
    // You can use it to print what you want.
    //
    // For example :
    //
    // 
    //  RhobanSSLAnnotation::Annotations get_annotations() const{
    //      RhobanSSLAnnotation::Annotations annotations;
    //      static double d = 0;
    //      d += 0.01;
    //
    //      annotations.addCircle(3, 3, 1, "cyan");
    //      annotations.addArrow(0, 0, cos(d), sin(d)*2, "magenta", true);
    //      return annotations;
    //  }
    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

};


};
};

#endif
