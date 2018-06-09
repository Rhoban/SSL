#include "jeremy.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/placer.h>
#include <strategy/prepare_kickoff.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/defensor.h>
#include <robot_behavior/striker.h>
#include <robot_behavior/robot_follower.h>
#include <robot_behavior/pass.h>
#include <robot_behavior/protect_ball.h>
#include <core/collection.h>
#include <core/print_collection.h>

#define SUPPORT "support"
#define STRIKER "striker"
#define GOALIE "goalie"
#define PASS "pass"
#define PROTECTBALL "protect_ball"

namespace RhobanSSL {
namespace Manager {

Jeremy::Jeremy(
    Ai::AiData & ai_data,
    const Referee & referee
):
    Manager(ai_data),
    referee(referee),
    last_referee_changement(0)
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
        Strategy::Prepare_kickoff::name,
        std::shared_ptr<Strategy::Strategy>(
            new Strategy::Prepare_kickoff(ai_data)
        )
    );
    register_strategy(
        GOALIE, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Goalie* goalie = new Robot_behavior::Goalie(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(goalie);
                }, false // it is not a goal
            )
        )
    );
    register_strategy(
        STRIKER, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Striker* striker = new Robot_behavior::Striker(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(striker);
                }, false // it is not a goal
            )
        )
    );
    register_strategy(
        SUPPORT, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::RobotFollower* f = new Robot_behavior::RobotFollower(ai_data);
                    f->declare_robot_to_follow(1, Vector2d(1.0, 1.0), Vision::Team::Ally);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(f);
                }, false // it is not a goal
            )
        )
    );
    register_strategy(
        PASS, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Pass* f = new Robot_behavior::Pass(ai_data);
                    f->declare_robot_to_pass(2, Vision::Team::Ally);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(f);
                }, false // it is not a goal
            )
        )
    );
    register_strategy(
        PROTECTBALL, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::ProtectBall* guard = new Robot_behavior::ProtectBall(ai_data);
                    guard->declare_radius(1);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(guard);
                }, false // it is not a goal
            )
        )
    );
    assign_strategy(
        Strategy::Halt::name, 0.0,
        get_team_ids()
    ); // TODO TIME !
}

void Jeremy::analyse_data(double time){
    // We change the point of view of the team
    change_team_and_point_of_view(
        referee.get_team_color( get_team_name() ),
        referee.blue_have_it_s_goal_on_positive_x_axis()
    );
    change_ally_and_opponent_goalie_id(
        referee.blue_goalie_id(),
        referee.yellow_goalie_id()
    );
}


void Jeremy::choose_a_strategy(double time){
    if( referee.edge_entropy() > last_referee_changement ){
        clear_strategy_assignement();
        if( referee.get_state() == Referee_Id::STATE_INIT ){
        } else if( referee.get_state() == Referee_Id::STATE_HALTED ){
            assign_strategy( Strategy::Halt::name, time, get_valid_team_ids() );
        } else if( referee.get_state() == Referee_Id::STATE_STOPPED ){
            if(get_valid_team_ids().size() > 0){
                if( not( get_strategy_<Strategy::Tare_and_synchronize>().is_tared_and_synchronized() ) ){
                    assign_strategy( Strategy::Tare_and_synchronize::name, time, get_valid_player_ids() );
                }else{
                    place_all_the_robots(time, future_strats);
                }
            }
        } else if( referee.get_state() == Referee_Id::STATE_PREPARE_KICKOFF ){
            if( get_team() == referee.kickoff_team() ){
                get_strategy_<Strategy::Prepare_kickoff>().set_kicking(true);
            }else{
                get_strategy_<Strategy::Prepare_kickoff>().set_kicking(false);
            }
            future_strats = {Strategy::Prepare_kickoff::name};
            declare_and_assign_next_strategies( future_strats );
        } else if( referee.get_state() == Referee_Id::STATE_PREPARE_PENALTY ){
        } else if( referee.get_state() == Referee_Id::STATE_RUNNING ){
            future_strats = { PROTECTBALL };
            declare_and_assign_next_strategies(future_strats);
        } else if( referee.get_state() == Referee_Id::STATE_TIMEOUT ){
            assign_strategy( Strategy::Halt::name, time, get_valid_team_ids() );
        }
        last_referee_changement = referee.edge_entropy();
    }
}

void Jeremy::update(double time){
    //update_strategies(time);
    update_current_strategies(time);
    analyse_data(time);
    choose_a_strategy(time);
}

Jeremy::~Jeremy(){ }

};
};
