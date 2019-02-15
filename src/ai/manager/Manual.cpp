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

#include "Manual.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/example.h>
#include <robot_behavior/example_machine_state.h>
#include <robot_behavior/defensor.h>
#include <robot_behavior/passive_defensor.h>
#include <robot_behavior/concept_proof_spinner.h>
#include <robot_behavior/test_kicker.h>
#include <robot_behavior/test_velocity_consign.h>
#include <robot_behavior/patrol.h>
#include <robot_behavior/position_follower.h>
#include <robot_behavior/striker.h>
#include <robot_behavior/striker_ai.h>
#include <robot_behavior/predict_futur.h>
#include <robot_behavior/obstructor.h>
#include <robot_behavior/test_infra.h>
#include <robot_behavior/pass_dribbler.h>
#include <robot_behavior/wait_pass.h>
#include <robot_behavior/pass.h>
#include <robot_behavior/tutorials/begginer/go_corner.h>
#include <robot_behavior/tutorials/begginer/goalie.h>
#include <robot_behavior/tutorials/begginer/defensor.h>
#include <robot_behavior/tutorials/begginer/robot_looks_a_given_position.h>
#include <robot_behavior/tutorials/intermediate/striker.h>
#include <robot_behavior/tutorials/intermediate/prepare_strike.h>
#include <robot_behavior/test_relative_velocity_consign.h>

namespace RhobanSSL {
namespace Manager {

Manual::Manual( Ai::AiData & ai_data ):
    Manager(ai_data),
    team_color(Ai::Team::Unknown),
    goal_to_positive_axis(true),
    ally_goalie_id(0),
    opponent_goalie_id(0)
{

    change_team_and_point_of_view(
        ai_data.team_color,
        goal_to_positive_axis
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
        "Robot looks at a given position", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Robot_looks_a_given_position* robot_looks_a_given_position = new Robot_behavior::Robot_looks_a_given_position(ai_data);
	            robot_looks_a_given_position->set_direction( M_PI/2.0 );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(robot_looks_a_given_position);
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
        "Example_machine_state", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Example_machine_state* ex = new Robot_behavior::Example_machine_state(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(ex);
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
        "two_way_trip", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* two_way_trip = Robot_behavior::Patrol::two_way_trip(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(two_way_trip);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "two_way_on_width_trip_ally", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* two_way_trip = Robot_behavior::Patrol::two_way_trip_on_width(ai_data, true);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(two_way_trip);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "two_way_on_width_trip_opponent", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* two_way_trip = Robot_behavior::Patrol::two_way_trip_on_width(ai_data, false);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(two_way_trip);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "two_way_trip_right", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* two_way_trip = Robot_behavior::Patrol::two_way_trip_on_border(ai_data, false);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(two_way_trip);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "two_way_trip_left", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* two_way_trip = Robot_behavior::Patrol::two_way_trip_on_border(ai_data, true);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(two_way_trip);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "tour_of_the_field", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* tour_of_the_field = Robot_behavior::Patrol::tour_of_the_field(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(tour_of_the_field);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "inverse_tour_of_the_field", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* tour_of_the_field = Robot_behavior::Patrol::tour_of_the_field(ai_data, true);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(tour_of_the_field);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_kicker", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_kicker* pt = new Robot_behavior::Test_kicker(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_NW_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_NW_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_NE_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_NE_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_SW_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_SW_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_SE_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_SE_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_rotation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_rotation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );

    register_strategy(
        "test_NE_rotation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_NE_rotation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_NW_rotation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_NW_rotation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );

    register_strategy(
        "test_SE_rotation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_SE_rotation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );

    register_strategy(
        "test_SW_rotation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_SW_rotation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_N_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_N_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_S_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_S_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_E_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_E_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_W_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_W_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );


    register_strategy(
        "test_NW_SE_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_NW_SE_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_SW_NW_translation_for_pid", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Patrol* pt = Robot_behavior::Patrol::test_SW_NW_translation_for_pid(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );


    register_strategy(
        "proof_concept_spinner", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Concept_proof_spinner* concept_proof_spinner = new Robot_behavior::Concept_proof_spinner(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(concept_proof_spinner);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Example", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Example* example = new Robot_behavior::Example(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(example);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Begginer Go corner", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Begginer_go_corner* go_corner = new Robot_behavior::Begginer_go_corner(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(go_corner);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Begginer Goalie", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Begginer_goalie* goalie = new Robot_behavior::Begginer_goalie(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(goalie);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Begginer Defensor", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Begginer_defensor* defensor = new Robot_behavior::Begginer_defensor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Intermediate Striker", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Intermediate_striker* striker = new Robot_behavior::Intermediate_striker(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(striker);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Intermediate Prepare to strike", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Intermediate_Prepare_strike* prepare_strike = new Robot_behavior::Intermediate_Prepare_strike(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(prepare_strike);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "Obstructor", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Obstructor* obstructor = new Robot_behavior::Obstructor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(obstructor);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "StrikerAi", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::StrikerAi* striker_ai = new Robot_behavior::StrikerAi(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(striker_ai);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "PredictFutur", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::PredictFutur* p = new Robot_behavior::PredictFutur(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(p);
                }, false
            )
        )
    );

    int velocity = 2.0;
    register_strategy(
        "test_N_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_velocity_consign* pt = Robot_behavior::Test_velocity_consign::get_N_movement(
                        ai_data, velocity
                    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_E_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_velocity_consign* pt = Robot_behavior::Test_velocity_consign::get_E_movement(
                        ai_data, velocity
                    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_W_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_velocity_consign* pt = Robot_behavior::Test_velocity_consign::get_W_movement(
                        ai_data, velocity
                    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_S_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_velocity_consign* pt = Robot_behavior::Test_velocity_consign::get_S_movement(
                        ai_data, velocity
                    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_NW_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_velocity_consign* pt = Robot_behavior::Test_velocity_consign::get_NW_movement(
                        ai_data, velocity
                    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_NE_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_velocity_consign* pt = Robot_behavior::Test_velocity_consign::get_NE_movement(
                        ai_data, velocity
                    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_SW_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_velocity_consign* pt = Robot_behavior::Test_velocity_consign::get_SW_movement(
                        ai_data, velocity
                    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );
    register_strategy(
        "test_SE_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_velocity_consign* pt = Robot_behavior::Test_velocity_consign::get_SE_movement(
                        ai_data, velocity
                    );
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(pt);
                }, false // we don't want to define a goal here !
            )
        )
    );



    register_strategy(
        "TestInfra", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::TestInfra* p = new Robot_behavior::TestInfra(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(p);
                }, false
            )
        )
    );
    register_strategy(
        "TestPassDribbler", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Pass_dribbler* p = new Robot_behavior::Pass_dribbler(ai_data);
                    p->declare_point_to_pass(ally_goal_center());
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(p);
                }, false
            )
        )
    );
    register_strategy(
        "TestPass", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Pass* p = new Robot_behavior::Pass(ai_data);
                    // p->declare_point_to_pass(ally_goal_center());
                    p->declare_robot_to_pass(2, Vision::Team::Ally);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(p);
                }, false
            )
        )
    );
    register_strategy(
        "WaitPass", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::WaitPass* p = new Robot_behavior::WaitPass(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(p);
                }, false
            )
        )
    );

    register_strategy(
        "test_angular_only_relative_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_relative_velocity_consign* p = Robot_behavior::Test_relative_velocity_consign::get_movement_angular_velocity_only(ai_data, M_PI/2);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(p);
                }, false // we don't want to define a goal here !
            )
        )
    );

    register_strategy(
        "test_linear_only_relative_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_relative_velocity_consign* p = Robot_behavior::Test_relative_velocity_consign::get_movement_linear_velocity_only(ai_data, Vector2d(0,1));
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(p);
                }, false // we don't want to define a goal here !
            )
        )
    );

    register_strategy(
        "test_linear_and_angular_relative_velocity_consign", std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Test_relative_velocity_consign* p = Robot_behavior::Test_relative_velocity_consign::get_movement_angular_and_linear_velocity(ai_data, Vector2d(0,1), M_PI*7/3);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(p);
                }, false // we don't want to define a goal here !
            )
        )
    );



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
    assign_strategy(
        Strategy::Halt::name, 0.0,
        get_team_ids()
   ); // TODO TIME !
   //strategy_was_assigned = false;
}


void Manual::assign_point_of_view_and_goalie(){
    // DEBUG(team_color);
    // DEBUG(Ai::Team::Yellow);
    change_team_and_point_of_view(
        team_color,
        goal_to_positive_axis
    );
}

void Manual::set_team_color( Ai::Team team_color ){
    this->team_color = team_color;
}

void Manual::define_goal_to_positive_axis(bool value){
    this->goal_to_positive_axis = value;
}


void Manual::update(double time){
    // if( not( get_strategy_<Strategy::Tare_and_synchronize>().is_tared_and_synchronized() ) ){
    //   assign_strategy( Strategy::Tare_and_synchronize::name, time, get_valid_player_ids() );
    // }
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
