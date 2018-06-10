#include "Manual_adrien.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/defensor.h>
#include <robot_behavior/passive_defensor.h>
#include <robot_behavior/position_follower.h>
#include <robot_behavior/striker.h>

namespace RhobanSSL {
namespace Manager {

Manual_adrien::Manual_adrien( Ai::AiData & ai_data ):
    Manager(ai_data),
    team_color(ai_data.team_color),
    goal_to_positive_axis(true),
    ally_goalie_id(0),
    oponnent_goalie_id(0)
{

    register_strategy(
        Strategy::Halt::name, std::shared_ptr<Strategy::Strategy>(
            new Strategy::Halt(ai_data)
        )
    );
    register_strategy(
        Strategy::Tare_and_synchronize::name,
        std::shared_ptr<Strategy::Strategy>(
            new Strategy::Tare_and_synchronize(ai_data)
        )
    );
    register_strategy(
        "Goalie", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Goalie* goalie = new Robot_behavior::Goalie(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(goalie);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Striker", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Striker* striker = new Robot_behavior::Striker(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(striker);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Defensor1", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Defensor* defensor = new Robot_behavior::Defensor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Defensor2", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Defensor* defensor = new Robot_behavior::Defensor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "passive_defensor", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Passive_defensor* passive_defensor = new Robot_behavior::Passive_defensor(ai_data);
                    passive_defensor->set_robot_to_obstacle( 0, Vision::Team::Ally );
                    passive_defensor->set_barycenter( 0.5 );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(passive_defensor);
                }, false // we don't want to define a goal here !
            )
        )
    );
    assign_strategy(
        Strategy::Halt::name, 0.0,
        get_team_ids()
   ); // TODO TIME !
   //strategy_was_assigned = false;
}

    
void Manual_adrien::assign_point_of_view_and_goalie(){
    change_team_and_point_of_view(
        team_color,
        goal_to_positive_axis
    );
}

void Manual_adrien::set_team_color( Ai::Team team_color ){
    this->team_color = team_color;
}

void Manual_adrien::define_goal_to_positive_axis(bool value){
    this->goal_to_positive_axis = goal_to_positive_axis;
}


void Manual_adrien::update(double time){
    //update_strategies(time);
    update_current_strategies(time);
    assign_point_of_view_and_goalie();
    //if( ! strategy_was_assigned ){
    //    assign_strategy(
    //        "Goalie",
    //        //"Position Follower",
    //        time, get_team_ids()
    //    );
	//    strategy_was_assigned = true;
    //}
}

Manual_adrien::~Manual_adrien(){ }

};
};
