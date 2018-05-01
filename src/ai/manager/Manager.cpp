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

void Manager::set_team( Ai::Team team ){
     this->team = team;
}
Ai::Team Manager::get_team() const{
     return team;
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


void Manager::change_team_point_of_view( bool blue_have_it_s_goal_on_positive_x_axis ){
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
            game_state.change_frame_for_all_objects(
                rhoban_geometry::Point(0.0,0.0),
                Vector2d(-1.0, 0.0), Vector2d(0.0, -1.0)
            );
        }else{
            game_state.change_frame_for_all_objects(
                rhoban_geometry::Point(0.0,0.0),
                Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)
            );
        }
    }

}

Manager::Manager( Ai::AiData& game_state ):
    blueIsNotSet(true),
    game_state(game_state)
{ }

Manager::~Manager(){ }

};
};
