#include "Manager.h"

#include <debug.h>
#include <strategy/halt.h>
#include <algorithm>
#include <core/collection.h>
#include <core/print_collection.h>
#include <algorithm>
#include <math/matching.h>

namespace RhobanSSL {
namespace Manager {

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
     return ai_data.team_color;
}
const std::string & Manager::get_team_name() const{
     return ai_data.team_name;
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

    assert( static_cast<unsigned int>(strategy.min_robots()) <= robot_ids.size() );

    strategy.set_goalie( goalie_id );
    strategy.set_goalie_opponent( goalie_opponent_id );
    strategy.set_robot_affectation( robot_ids );
    strategy.start(time);

    std::cout << "Manager: Assigning " << strategy.get_robot_ids() << " to " <<
        robot_ids.size() << " robots : " << "(goalie : "<< strategy.get_goalie() << ")" << std::endl;
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
            [&](
                int id, 
                std::shared_ptr<Robot_behavior::RobotBehavior> behavior
            ){
                #if 0 // HACK : TODO  -- quick fix
                DEBUG("Strategy : " << name);
                DEBUG("Current assignation : " << id);
                #ifndef NDEBUG
                bool id_is_present = false;
                for( int robot_id : this->get_strategy(name).get_robot_ids() ){
                    if( robot_id == id ){
                        id_is_present = true;
                        break;
                    }
                }
                DEBUG("Robot presence : " << id_is_present);
                assert( id_is_present );
                #endif
                #endif
                return robot_behaviors[id] = behavior; 
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
        ai_data.change_team_color( team );
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
            ai_data.change_frame_for_all_objects(
                rhoban_geometry::Point(0.0,0.0),
                Vector2d(-1.0, 0.0), Vector2d(0.0, -1.0)
            );
        }else{
            ai_data.change_frame_for_all_objects(
                rhoban_geometry::Point(0.0,0.0),
                Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)
            );
        }
    }

}

Manager::Manager( Ai::AiData& ai_data ):
    blueIsNotSet(true),
    goalie_id(-1),
    goalie_opponent_id(-1),
    ai_data(ai_data)
{
    register_strategy(
        MANAGER__REMOVE_ROBOTS, std::shared_ptr<Strategy::Strategy>(
            new Strategy::Halt(ai_data)
        )
    );
    register_strategy(
        MANAGER__PLACER,
        std::shared_ptr<Strategy::Strategy>(
            new Strategy::Placer(ai_data)
        )
    );
}

int Manager::time() const {
    return ai_data.time;
}

int Manager::dt() const {
    return ai_data.dt;
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
    // TODO : we need to detect when the list of invalid robot change.
    // When it change, then, we need to reaffect robot ids.    

    int nb_valid = 0;
    int n_robots = team_ids.size();
    for(int i=0; i<n_robots; i++ ){
        if( ai_data.robot_is_valid( team_ids[i] ) ){
            nb_valid ++ ;
        }
    }
    valid_team_ids.clear();
    valid_player_ids.clear();

    invalid_team_ids.clear();
    for(int i=0; i<n_robots; i++ ){
        int id = team_ids[i];
        if( ai_data.robot_is_valid( id ) ){
            valid_team_ids.push_back( id );
            if( goalie_id != id ){
                valid_player_ids.push_back( id );
            }
        }else{
            invalid_team_ids.push_back( id );
        }
    }
}

const std::vector<int> & Manager::get_valid_team_ids() const {
    return valid_team_ids;
}
const std::vector<int> & Manager::get_valid_player_ids() const {
    return valid_player_ids;
}
const std::vector<int> & Manager::get_invalid_team_ids() const {
    return invalid_team_ids;
}
Manager::~Manager(){ }


std::vector<std::string> Manager::get_available_strategies()
{
    std::vector<std::string> strategyNames;

    for (auto &entry : strategies) {
        strategyNames.push_back(entry.first);
    }

    return strategyNames;
}

void Manager::aggregate_all_starting_position_of_all_strategies(){
    starting_positions.clear();
    repartitions_of_starting_positions.clear();

    // For the players
    goal_has_to_be_placed = false;
    for( const std::string & strategy_name : next_strategies ){
        std::list<
            std::pair<rhoban_geometry::Point,ContinuousAngle>
        > starts = get_strategy(
            strategy_name
        ).get_starting_positions( get_valid_team_ids().size() );
        
        starting_positions.insert( starting_positions.end(), starts.begin(), starts.end() );
        
        repartitions_of_starting_positions.push_back(
            std::pair<std::string, int>( strategy_name, starts.size() )
        );

        // For the goalie
        rhoban_geometry::Point goalie_linear_position;
        ContinuousAngle goalie_angular_position;
        if(
            get_strategy(strategy_name).get_starting_position_for_goalie(
                goalie_linear_position, goalie_angular_position
            )
        ){
            assert( not(goal_has_to_be_placed) ); // Two goal is defined, check you are not assigning two stratgies with a goal ! 
            this->goalie_linear_position = goalie_linear_position;
            this->goalie_angular_position = goalie_angular_position;
            goal_has_to_be_placed = true;
        }
    }
}

void Manager::declare_robot_positions_in_the_placer(){
    if(goal_has_to_be_placed){
        get_strategy_<
            Strategy::Placer
        >().set_goalie_positions(
            goalie_linear_position,
            goalie_angular_position
        );
    }else{
        get_strategy_<
            Strategy::Placer
        >().ignore_goalie();
    }

    assert( starting_positions.size() <=  get_valid_team_ids().size() );

    get_strategy_<Strategy::Placer>().set_positions(
        robot_affectations, robot_consigns
    );
}

void Manager::place_all_the_robots(
    double time, const std::list<std::string> & next_strategies
){
    declare_next_strategies(next_strategies);
    declare_robot_positions_in_the_placer();
    assign_strategy( 
        Strategy::Placer::name, time, 
        get_valid_team_ids()
    );
}

const std::vector<int> & Manager::get_robot_affectations( const std::string & strategy_name ) const {
    assert(
        robot_affectations_by_strategy.find(strategy_name) != 
        robot_affectations_by_strategy.end()
    );
    return robot_affectations_by_strategy.at(strategy_name);
}

void Manager::declare_next_strategies(const std::list<std::string> & next_strategies){
    this->next_strategies = next_strategies;
    aggregate_all_starting_position_of_all_strategies();
    determine_the_robot_needs_for_the_strategies();
    sort_robot_ordered_by_the_distance_with_starting_position();
    compute_robot_affectations_to_strategies();
}


    
void Manager::determine_the_robot_needs_for_the_strategies(){
    nb_of_extra_robots = (
        get_valid_player_ids().size() - starting_positions.size()
    ); 
    nb_of_extra_robots_non_affected = (
        get_valid_player_ids().size() - starting_positions.size()
    ); 
    robot_affectations_by_strategy.clear();
    number_of_extra_robot_by_strategy.clear();
    for( const std::pair<std::string, int> & elem : repartitions_of_starting_positions ){
        const std::string & strategy_name = elem.first; 
        const int nb_robots = elem.second;

        unsigned int extra_robots = 0;
        if( get_strategy(strategy_name).max_robots()==-1 ){
            extra_robots = nb_of_extra_robots_non_affected;
            nb_of_extra_robots_non_affected = 0;
        }else{
            extra_robots = std::min(
                nb_of_extra_robots_non_affected, 
                static_cast<unsigned int>( get_strategy(strategy_name).max_robots() ) 
            );
            nb_of_extra_robots_non_affected -= extra_robots;
        }
        number_of_extra_robot_by_strategy[strategy_name] = extra_robots;

        robot_affectations_by_strategy[strategy_name] = std::vector<int>( nb_robots+extra_robots );
    }
}

void Manager::sort_robot_ordered_by_the_distance_with_starting_position(){
    assert( starting_positions.size() <= get_valid_player_ids().size() );
    robot_consigns = std::vector< 
        std::pair<rhoban_geometry::Point, ContinuousAngle> 
    >( get_valid_player_ids().size() );
    robot_affectations.resize( get_valid_player_ids().size() );

    std::vector<
        std::pair<rhoban_geometry::Point, ContinuousAngle>
    > choising_positions = list2vector(
        starting_positions
    );

    std::function<
        double( 
            const int & robot_id,
            const std::pair<rhoban_geometry::Point, ContinuousAngle> & pos
        ) 
    > robot_ranking = [this](
        const int & robot_id,
        const std::pair<rhoban_geometry::Point, ContinuousAngle> & pos 
    ){
        return Vector2d(
            pos.first -
            this->robot(robot_id).get_movement().linear_position(time())
        ).norm_square();
    };

    std::function<
        double( 
            const std::pair<rhoban_geometry::Point, ContinuousAngle> & pos,
            const int & robot_id
        ) 
    > distance_ranking = [this](
        const std::pair<rhoban_geometry::Point, ContinuousAngle> & pos, 
        const int & robot_id
    ){
        return Vector2d(
            pos.first -
            this->robot(robot_id).get_movement().linear_position(time())
        ).norm_square();
    };

    matching::Matchings matchings = matching::gale_shapley_algorithm(
        get_valid_player_ids(), choising_positions,
        robot_ranking, distance_ranking,
        false, false
    );

    std::list <int> not_choosen_robot;
    for( unsigned int id : matchings.unaffected_man ){
        not_choosen_robot.push_back( get_valid_player_ids()[id] );
    }
     
    for(
        unsigned int i=0; i<choising_positions.size(); i++
    ){
        const std::pair<rhoban_geometry::Point,ContinuousAngle> & pos = choising_positions[i];
        robot_consigns[i] = pos; 
        robot_affectations[i] = get_valid_player_ids()[
            matchings.women_to_man_matchings.at(i)
        ];
    }


    // TODO : have a better default placer !
    std::list<int>::const_iterator it = not_choosen_robot.begin();
    for( unsigned int i=starting_positions.size(); i<get_valid_player_ids().size(); i++ ){
        robot_consigns[i] = std::pair<rhoban_geometry::Point, ContinuousAngle>(
            rhoban_geometry::Point(
                -2.0, 
                (
                    2*ai_data.constants.robot_radius 
                    +
                    8*ai_data.constants.radius_ball/2.0
                )*(
                    (i-starting_positions.size()) - 
                    (get_valid_player_ids().size()-starting_positions.size())*.5
                )
            ),
            ContinuousAngle(0.0)
        ); 
        robot_affectations[i] = *it;
        it++;
    }
}

void Manager::compute_robot_affectations_to_strategies(){
    // TODO MANAGE THE CASE OF THE GOAL !
    unsigned int cpt_robot = 0;
    unsigned int cpt_extra_robot = 0;
    for(
        const std::pair<std::string, int> & elem : 
        repartitions_of_starting_positions
    ){
        const std::string & strategy_name = elem.first; 
        const int nb_robots = elem.second;
        for( int i=0; i<nb_robots; i++ ){
            robot_affectations_by_strategy[strategy_name][i] = robot_affectations.at(cpt_robot+i);
        }
        unsigned int extra_robots = number_of_extra_robot_by_strategy.at( strategy_name );

        for( unsigned int i=0; i<extra_robots; i++ ){
            unsigned int offset = starting_positions.size();
            robot_affectations_by_strategy[strategy_name][nb_robots+i] = robot_affectations.at(i + offset + cpt_extra_robot);
        }
        cpt_robot += nb_robots;
        cpt_extra_robot += extra_robots;
    }
}


Ai::Robot& Manager::robot( int robot_number) const {
    return ai_data.robots.at( Vision::Team::Ally ).at( robot_number );
}

};
};
