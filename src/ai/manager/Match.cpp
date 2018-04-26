#include "Match.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/sandbox.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>

namespace RhobanSSL {
namespace Manager {

Match::Match(
    Ai::AiData & game_state,
    const Referee & referee
):
    game_state(game_state),
    referee(referee)
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
        Strategy::Sandbox::name,
        std::shared_ptr<Strategy::Strategy>(
            new Strategy::Sandbox(game_state)
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
    sandbox = false;
    start = -1.0;
    assign_strategy(
        Strategy::Halt::name, 0.0, 
        get_team_ids()
   ); // TODO TIME !
}

void Match::analyse_data(double time){
}
void Match::choose_a_strategy(double time){
    if(start==-1.0){ 
        assign_strategy(
            Strategy::Tare_and_synchronize::name, time,
            get_team_ids()
        );
        start = time;
        return;
    }

    if( 
        ! get_strategy<Strategy::Tare_and_synchronize>().is_tared_and_synchronized()
    ){
        return;
    }

    if( ! sandbox ){
        assign_strategy(
            Strategy::Sandbox::name, time,
            get_team_ids()
        );
//        assign_strategy(
//            "Goalie", time,
//            get_team_ids()
//        );
        sandbox = true;
    }
}

void Match::update(double time){
    //update_strategies(time);
    update_current_strategy(time);
    analyse_data(time);
    choose_a_strategy(time);
}

Match::~Match(){ }

};
};
