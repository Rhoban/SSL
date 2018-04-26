#include "Manual.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/sandbox.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>

namespace RhobanSSL {
namespace Manager {

Manual::Manual(
    Ai::AiData & game_state
):
    game_state(game_state)
{

    register_strategy(
        Strategy::Halt::name, std::shared_ptr<Strategy::Strategy>(
            new Strategy::Halt() 
        )
    );
    register_strategy(
        Strategy::Tare_and_synchronize::name,
        std::shared_ptr<Strategy::Strategy>( 
            new Strategy::Tare_and_synchronize()
        )
    );
    register_strategy(
        "Goalie", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                [&](double time, double dt){
                    Goalie* goalie = new Goalie(
                        game_state.constants.left_post_position, 
                        game_state.constants.right_post_position, 
                        game_state.constants.waiting_goal_position, 
                        game_state.constants.penalty_rayon, 
                        game_state.constants.robot_radius,
                        time, dt
                    );
                    goalie->set_translation_pid( 
                        game_state.constants.p_translation,
                        game_state.constants.i_translation, 
                        game_state.constants.d_translation
                    );
                    goalie->set_orientation_pid(
                        game_state.constants.p_orientation,
                        game_state.constants.i_orientation, 
                        game_state.constants.d_orientation
                    );
                    goalie->set_limits(
                        game_state.constants.translation_velocity_limit,
                        game_state.constants.rotation_velocity_limit
                    );
                    return std::shared_ptr<RobotBehavior>(goalie);
                }, true
            )
        )
    );
    register_strategy(
        "Position Follower", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                [&](double time, double dt){
		    PositionFollower* follower = new PositionFollower( time, dt );
		    follower->set_following_position(
			Eigen::Vector2d(-2.0, 1.0), ContinuousAngle(M_PI/2.0)
		    );
		    follower->set_translation_pid(
			game_state.constants.p_translation,
			game_state.constants.i_translation, 
			game_state.constants.d_translation
		    );
		    follower->set_orientation_pid(
			game_state.constants.p_orientation, game_state.constants.i_orientation, 
			game_state.constants.d_orientation
		    );
		    follower->set_limits(
			game_state.constants.translation_velocity_limit,
			game_state.constants.rotation_velocity_limit
		    );
                    return std::shared_ptr<RobotBehavior>(follower);
                }
            )
        )
    );
    assign_strategy(
        Strategy::Halt::name, 0.0, 
        get_team_ids()
   ); // TODO TIME !
   strategy_was_assigned = false;
}

void Manual::update(double time){
    //update_strategies(time);
    update_current_strategy(time);
    if( ! strategy_was_assigned ){
        assign_strategy(
            "Goalie",
            //"Position Follower",
            time, get_team_ids()
        );
	strategy_was_assigned = true;
    }
}

Manual::~Manual(){ }

};
};
