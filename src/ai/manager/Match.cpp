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

#include "Match.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/placer.h>
#include <strategy/prepare_kickoff.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/defensor.h>
#include <robot_behavior/striker.h>
#include <core/collection.h>
#include <core/print_collection.h>

#define GOALIE "Goalie"
#define DEFENSOR1 "Defensor1"
#define DEFENSOR2 "Defensor2"
#define STRIKER "Striker"

namespace RhobanSSL {
namespace Manager {

Match::Match(
    Ai::AiData & ai_data,
    const GameState & game_state
):
    Manager(ai_data),
    game_state(game_state),
    last_game_state_changement(0)
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
                }, true //it is a goal
            )
        )
    );
    register_strategy(
        DEFENSOR1, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Defensor* defensor = new Robot_behavior::Defensor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
                }, false // it is not a goal
            )
        )
    );
    register_strategy(
        DEFENSOR2, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Defensor* defensor = new Robot_behavior::Defensor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
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
    assign_strategy(
        Strategy::Halt::name, 0.0,
        get_team_ids()
    ); // TODO TIME !
}


void Match::choose_a_strategy(double time){
    if( game_state.edge_entropy() > last_game_state_changement ){
        clear_strategy_assignement();
        if( game_state.get_state() == state_name::halt ){
            assign_strategy( Strategy::Halt::name, time, get_valid_team_ids() );
        } else if( game_state.get_state() == state_name::stop ){
            if(get_valid_team_ids().size() > 0){
                if( not( get_strategy_<Strategy::Tare_and_synchronize>().is_tared_and_synchronized() ) ){
                    assign_strategy( Strategy::Tare_and_synchronize::name, time, get_valid_player_ids() );
                }else{
                    place_all_the_robots(time, future_strats);
                }
            }
        } else if( game_state.get_state() == state_name::STATE_PREPARE_KICKOFF ){
            if( get_team() == game_state.kickoff_team() ){
                get_strategy_<Strategy::Prepare_kickoff>().set_kicking(true);
            }else{
                get_strategy_<Strategy::Prepare_kickoff>().set_kicking(false);
            }
            future_strats = {Strategy::Prepare_kickoff::name};
            declare_and_assign_next_strategies( future_strats );
        } else if( game_state.get_state() == state_name::STATE_PREPARE_PENALTY ){
        } else if( game_state.get_state() == state_name::STATE_RUNNING ){
            future_strats = { GOALIE, DEFENSOR1, STRIKER };
            declare_and_assign_next_strategies(future_strats);
        } else if( game_state.get_state() == state_name::STATE_TIMEOUT ){
            assign_strategy( Strategy::Halt::name, time, get_valid_team_ids() );
        }
        last_game_state_changement = game_state.edge_entropy();
    }
}

void Match::update(double time){
    //update_strategies(time);
    update_current_strategies(time);
    choose_a_strategy(time);
}

Match::~Match(){ }

};
};
