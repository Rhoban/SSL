#include "Manual.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/position_follower.h>

namespace RhobanSSL {
namespace Manager {

Manual::Manual( Ai::AiData & ai_data ):
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
        "Position Follower", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
		    Robot_behavior::PositionFollower* follower = new Robot_behavior::PositionFollower( ai_data, time, dt );
		    follower->set_following_position(
			Vector2d(-2.0, 1.0), ContinuousAngle(M_PI/2.0)
		    );
		    follower->set_translation_pid(
			ai_data.constants.p_translation,
			ai_data.constants.i_translation,
			ai_data.constants.d_translation
		    );
		    follower->set_orientation_pid(
			ai_data.constants.p_orientation, ai_data.constants.i_orientation,
			ai_data.constants.d_orientation
		    );
		    follower->set_limits(
                ai_data.constants.translation_velocity_limit,
                ai_data.constants.rotation_velocity_limit,
                ai_data.constants.translation_acceleration_limit,
                ai_data.constants.rotation_acceleration_limit
		    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(follower);
                }
            )
        )
    );
    assign_strategy(
        Strategy::Halt::name, 0.0,
        get_team_ids()
   ); // TODO TIME !
   //strategy_was_assigned = false;
}

    
void Manual::assign_point_of_view_and_goalie(){
    change_team_and_point_of_view(
        team_color,
        goal_to_positive_axis
    );
}

void Manual::set_team_color( Ai::Team team_color ){
    this->team_color = team_color;
}

void Manual::define_goal_to_positive_axis(bool value){
    this->goal_to_positive_axis = goal_to_positive_axis;
}


void Manual::update(double time){
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

Manual::~Manual(){ }

};
};
