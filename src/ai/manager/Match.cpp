#include "Match.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/sandbox.h>
#include <strategy/prepare_kickoff.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>

namespace RhobanSSL {
namespace Manager {

Match::Match(
    Ai::AiData & game_state,
    const Referee & referee
):
    game_state(game_state),
    referee(referee),
    last_referee_changement(0)
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
        Strategy::Prepare_kickoff::name,
        std::shared_ptr<Strategy::Strategy>(
            new Strategy::Prepare_kickoff(game_state)
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
    assign_strategy(
        Strategy::Halt::name, 0.0, 
        get_team_ids()
   ); // TODO TIME !
}

void Match::analyse_data(double time){
}
void Match::choose_a_strategy(double time){
    if( referee.edge_entropy() > last_referee_changement ){
        if( referee.get_state() == Referee_Id::STATE_INIT ){
        } else if( referee.get_state() == Referee_Id::STATE_HALTED ){
            assign_strategy( Strategy::Halt::name, time, get_team_ids() );
        } else if( referee.get_state() == Referee_Id::STATE_STOPPED ){
        } else if( referee.get_state() == Referee_Id::STATE_PREPARE_KICKOFF ){
            assign_strategy( Strategy::Prepare_kickoff::name, time, get_team_ids() );
            if( get_team() == referee.kickoff_team() ){
                get_strategy<Strategy::Prepare_kickoff>().set_kicking(true);
            }else{
                get_strategy<Strategy::Prepare_kickoff>().set_kicking(false);
            }
        } else if( referee.get_state() == Referee_Id::STATE_PREPARE_PENALTY ){
        } else if( referee.get_state() == Referee_Id::STATE_RUNNING ){
        } else if( referee.get_state() == Referee_Id::STATE_TIMEOUT ){
        }
        last_referee_changement = referee.edge_entropy();
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
