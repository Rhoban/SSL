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

#include "manual.h"

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
#include <robot_behavior/tutorials/beginner/goto_ball.h>
#include <robot_behavior/tutorials/beginner/go_corner.h>
#include <robot_behavior/tutorials/beginner/goalie.h>
#include <robot_behavior/tutorials/medium/defender.h>
#include <robot_behavior/tutorials/beginner/see_ball.h>
#include <robot_behavior/tutorials/beginner/see_robot.h>
#include <robot_behavior/tutorials/beginner/annotations_robot_coords.h>
#include <robot_behavior/tutorials/beginner/robot_near_ball.h>
#include <robot_behavior/tutorials/beginner/robot_have_ball.h>
#include <robot_behavior/tutorials/beginner/annotations_ball_position.h>
#include <robot_behavior/tutorials/medium/striker.h>
#include <robot_behavior/tutorials/medium/prepare_strike.h>
#include <robot_behavior/test_relative_velocity_consign.h>

namespace rhoban_ssl
{
namespace manager
{
Manual::Manual(ai::AiData& ai_data)
  : Manager(ai_data)
  , team_color_(ai::Team::Unknown)
  , goal_to_positive_axis_(true)
  , ally_goalie_id_(0)
  , opponent_goalie_id_(0)
{
  changeTeamAndPointOfView(ai_data.team_color, goal_to_positive_axis_);

  registerStrategy("Goalie", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                 ai_data,
                                 [&](double time, double dt) {
                                   robot_behavior::Goalie* goalie = new robot_behavior::Goalie(ai_data);
                                   return std::shared_ptr<robot_behavior::RobotBehavior>(goalie);
                                 },
                                 false  // we don't want to define a goal here !
                                 )));
  registerStrategy("Striker", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                  ai_data,
                                  [&](double time, double dt) {
                                    robot_behavior::Striker* striker = new robot_behavior::Striker(ai_data);
                                    return std::shared_ptr<robot_behavior::RobotBehavior>(striker);
                                  },
                                  false  // we don't want to define a goal here !
                                  )));
  registerStrategy("Example_machine_state", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                ai_data,
                                                [&](double time, double dt) {
                                                  robot_behavior::ExampleMachineState* ex =
                                                      new robot_behavior::ExampleMachineState(ai_data);
                                                  return std::shared_ptr<robot_behavior::RobotBehavior>(ex);
                                                },
                                                false  // we don't want to define a goal here !
                                                )));
  registerStrategy("Defensor1", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                    ai_data,
                                    [&](double time, double dt) {
                                      robot_behavior::Defensor* defensor = new robot_behavior::Defensor(ai_data);
                                      return std::shared_ptr<robot_behavior::RobotBehavior>(defensor);
                                    },
                                    false  // we don't want to define a goal here !
                                    )));
  registerStrategy("passive_defensor", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                           ai_data,
                                           [&](double time, double dt) {
                                             robot_behavior::PassiveDefensor* passive_defensor =
                                                 new robot_behavior::PassiveDefensor(ai_data);
                                             passive_defensor->set_robot_to_obstacle(0, vision::Team::Ally);
                                             passive_defensor->set_barycenter(0.5);
                                             return std::shared_ptr<robot_behavior::RobotBehavior>(passive_defensor);
                                           },
                                           false  // we don't want to define a goal here !
                                           )));
  registerStrategy("Defensor2", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                    ai_data,
                                    [&](double time, double dt) {
                                      robot_behavior::Defensor* defensor = new robot_behavior::Defensor(ai_data);
                                      return std::shared_ptr<robot_behavior::RobotBehavior>(defensor);
                                    },
                                    false  // we don't want to define a goal here !
                                    )));
  registerStrategy("two_way_trip", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                       ai_data,
                                       [&](double time, double dt) {
                                         robot_behavior::Patrol* two_way_trip =
                                             robot_behavior::Patrol::twoWayTrip(ai_data);
                                         return std::shared_ptr<robot_behavior::RobotBehavior>(two_way_trip);
                                       },
                                       false  // we don't want to define a goal here !
                                       )));
  registerStrategy("two_way_on_width_trip_ally",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::Patrol* two_way_trip =
                             robot_behavior::Patrol::twoWayTripOnWidth(ai_data, true);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(two_way_trip);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("two_way_on_width_trip_opponent",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::Patrol* two_way_trip =
                             robot_behavior::Patrol::twoWayTripOnWidth(ai_data, false);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(two_way_trip);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("two_way_trip_right", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                             ai_data,
                                             [&](double time, double dt) {
                                               robot_behavior::Patrol* two_way_trip =
                                                   robot_behavior::Patrol::twoWayTripOnBorder(ai_data, false);
                                               return std::shared_ptr<robot_behavior::RobotBehavior>(two_way_trip);
                                             },
                                             false  // we don't want to define a goal here !
                                             )));
  registerStrategy("two_way_trip_left", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                            ai_data,
                                            [&](double time, double dt) {
                                              robot_behavior::Patrol* two_way_trip =
                                                  robot_behavior::Patrol::twoWayTripOnBorder(ai_data, true);
                                              return std::shared_ptr<robot_behavior::RobotBehavior>(two_way_trip);
                                            },
                                            false  // we don't want to define a goal here !
                                            )));
  registerStrategy("tour_of_the_field", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                            ai_data,
                                            [&](double time, double dt) {
                                              robot_behavior::Patrol* tour_of_the_field =
                                                  robot_behavior::Patrol::tourOfTheField(ai_data);
                                              return std::shared_ptr<robot_behavior::RobotBehavior>(tour_of_the_field);
                                            },
                                            false  // we don't want to define a goal here !
                                            )));
  registerStrategy("inverse_tour_of_the_field",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::Patrol* tour_of_the_field =
                             robot_behavior::Patrol::tourOfTheField(ai_data, true);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(tour_of_the_field);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("test_kicker", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                      ai_data,
                                      [&](double time, double dt) {
                                        robot_behavior::TestKicker* pt = new robot_behavior::TestKicker(ai_data);
                                        return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                      },
                                      false  // we don't want to define a goal here !
                                      )));
  registerStrategy("test_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::Patrol* pt =
                                                         robot_behavior::Patrol::testTranslationForPid(ai_data);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));
  registerStrategy("test_NW_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                      ai_data,
                                                      [&](double time, double dt) {
                                                        robot_behavior::Patrol* pt =
                                                            robot_behavior::Patrol::testNWTranslationForPid(ai_data);
                                                        return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                      },
                                                      false  // we don't want to define a goal here !
                                                      )));
  registerStrategy("test_NE_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                      ai_data,
                                                      [&](double time, double dt) {
                                                        robot_behavior::Patrol* pt =
                                                            robot_behavior::Patrol::testNETranslationForPid(ai_data);
                                                        return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                      },
                                                      false  // we don't want to define a goal here !
                                                      )));
  registerStrategy("test_SW_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                      ai_data,
                                                      [&](double time, double dt) {
                                                        robot_behavior::Patrol* pt =
                                                            robot_behavior::Patrol::testSWTranslationForPid(ai_data);
                                                        return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                      },
                                                      false  // we don't want to define a goal here !
                                                      )));
  registerStrategy("test_SE_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                      ai_data,
                                                      [&](double time, double dt) {
                                                        robot_behavior::Patrol* pt =
                                                            robot_behavior::Patrol::testSETranslationForPid(ai_data);
                                                        return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                      },
                                                      false  // we don't want to define a goal here !
                                                      )));
  registerStrategy("test_rotation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                ai_data,
                                                [&](double time, double dt) {
                                                  robot_behavior::Patrol* pt =
                                                      robot_behavior::Patrol::testRotationForPid(ai_data);
                                                  return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                },
                                                false  // we don't want to define a goal here !
                                                )));

  registerStrategy("test_NE_rotation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::Patrol* pt =
                                                         robot_behavior::Patrol::testNERotationForPid(ai_data);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));
  registerStrategy("test_NW_rotation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::Patrol* pt =
                                                         robot_behavior::Patrol::testNWRotationForPid(ai_data);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));

  registerStrategy("test_SE_rotation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::Patrol* pt =
                                                         robot_behavior::Patrol::testSERotationForPid(ai_data);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));

  registerStrategy("test_SW_rotation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::Patrol* pt =
                                                         robot_behavior::Patrol::testSWRotationForPid(ai_data);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));
  registerStrategy("test_N_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                     ai_data,
                                                     [&](double time, double dt) {
                                                       robot_behavior::Patrol* pt =
                                                           robot_behavior::Patrol::testNTranslationForPid(ai_data);
                                                       return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                     },
                                                     false  // we don't want to define a goal here !
                                                     )));
  registerStrategy("test_S_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                     ai_data,
                                                     [&](double time, double dt) {
                                                       robot_behavior::Patrol* pt =
                                                           robot_behavior::Patrol::testSTranslationForPid(ai_data);
                                                       return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                     },
                                                     false  // we don't want to define a goal here !
                                                     )));
  registerStrategy("test_E_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                     ai_data,
                                                     [&](double time, double dt) {
                                                       robot_behavior::Patrol* pt =
                                                           robot_behavior::Patrol::testETranslationForPid(ai_data);
                                                       return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                     },
                                                     false  // we don't want to define a goal here !
                                                     )));
  registerStrategy("test_W_translation_for_pid", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                     ai_data,
                                                     [&](double time, double dt) {
                                                       robot_behavior::Patrol* pt =
                                                           robot_behavior::Patrol::testWTranslationForPid(ai_data);
                                                       return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                     },
                                                     false  // we don't want to define a goal here !
                                                     )));

  registerStrategy("test_NW_SE_translation_for_pid",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::Patrol* pt = robot_behavior::Patrol::testNwSeTranslationForPid(ai_data);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("test_SW_NW_translation_for_pid",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::Patrol* pt = robot_behavior::Patrol::testSwNwTranslationForPid(ai_data);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                       },
                       false  // we don't want to define a goal here !
                       )));

  registerStrategy("proof_concept_spinner",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::ConceptProofSpinner* concept_proof_spinner =
                             new robot_behavior::ConceptProofSpinner(ai_data);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(concept_proof_spinner);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("Example", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                  ai_data,
                                  [&](double time, double dt) {
                                    robot_behavior::Example* example = new robot_behavior::Example(ai_data);
                                    return std::shared_ptr<robot_behavior::RobotBehavior>(example);
                                  },
                                  false  // we don't want to define a goal here !
                                  )));
  registerStrategy("Beginner go to ball",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::Beginner::Goto_ball* beginner_goto_ball =
                             new robot_behavior::Beginner::Goto_ball(ai_data);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(beginner_goto_ball);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("Beginner - Goalie", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                            ai_data,
                                            [&](double time, double dt) {
                                              robot_behavior::beginner::Goalie* goalie =
                                                  new robot_behavior::beginner::Goalie(ai_data);
                                              return std::shared_ptr<robot_behavior::RobotBehavior>(goalie);
                                            },
                                            false  // we don't want to define a goal here !
                                            )));
  registerStrategy("Medium - Defender", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                            ai_data,
                                            [&](double time, double dt) {
                                              robot_behavior::medium::Defender* defender =
                                                  new robot_behavior::medium::Defender(ai_data);
                                              return std::shared_ptr<robot_behavior::RobotBehavior>(defender);
                                            },
                                            false  // we don't want to define a goal here !
                                            )));
  registerStrategy("Beginner Annotations - Ball position",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::BeginnerAnnotationsBallPosition* ball_position =
                             new robot_behavior::BeginnerAnnotationsBallPosition(ai_data);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(ball_position);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("Beginner - See ball", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                              ai_data,
                                              [&](double time, double dt) {
                                                robot_behavior::beginner::SeeBall* see_ball =
                                                    new robot_behavior::beginner::SeeBall(ai_data);
                                                return std::shared_ptr<robot_behavior::RobotBehavior>(see_ball);
                                              },
                                              false  // we don't want to define a goal here !
                                              )));
  registerStrategy("Beginner - See Robot 3", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                 ai_data,
                                                 [&](double time, double dt) {
                                                   robot_behavior::Beginner::SeeRobot* see_robot =
                                                       new robot_behavior::Beginner::SeeRobot(ai_data);
                                                   see_robot->setRobotIdToSee(3);
                                                   return std::shared_ptr<robot_behavior::RobotBehavior>(see_robot);
                                                 },
                                                 false  // we don't want to define a goal here !
                                                 )));
  registerStrategy("Beginner - Robot near ball", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                     ai_data,
                                                     [&](double time, double dt) {
                                                       robot_behavior::BeginnerRobotNearBall* near_ball =
                                                           new robot_behavior::BeginnerRobotNearBall(ai_data);
                                                       return std::shared_ptr<robot_behavior::RobotBehavior>(near_ball);
                                                     },
                                                     false  // we don't want to define a goal here !
                                                     )));
  registerStrategy("Beginner - Robot have ball", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                     ai_data,
                                                     [&](double time, double dt) {
                                                       robot_behavior::BeginnerRobotHaveBall* have_ball =
                                                           new robot_behavior::BeginnerRobotHaveBall(ai_data);
                                                       return std::shared_ptr<robot_behavior::RobotBehavior>(have_ball);
                                                     },
                                                     false  // we don't want to define a goal here !
                                                     )));
  registerStrategy("Intermediate Striker", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                               ai_data,
                                               [&](double time, double dt) {
                                                 robot_behavior::IntermediateStriker* striker =
                                                     new robot_behavior::IntermediateStriker(ai_data);
                                                 return std::shared_ptr<robot_behavior::RobotBehavior>(striker);
                                               },
                                               false  // we don't want to define a goal here !
                                               )));

  register_strategy("Begginer - Annotations robot coords",
                    std::shared_ptr<Strategy::Strategy>(new Strategy::From_robot_behavior(
                        ai_data,
                        [&](double time, double dt) {
                          Robot_behavior::Beginner::AnnotationsRobotCoords* annot =
                              new Robot_behavior::Beginner::AnnotationsRobotCoords(ai_data);
                          return std::shared_ptr<Robot_behavior::RobotBehavior>(annot);
                        },
                        false  // we don't want to define a goal here !
                        )));
  registerStrategy("Intermediate Prepare to strike",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::IntermediatePrepareStrike* prepare_strike =
                             new robot_behavior::IntermediatePrepareStrike(ai_data);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(prepare_strike);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("Obstructor", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                     ai_data,
                                     [&](double time, double dt) {
                                       robot_behavior::Obstructor* obstructor = new robot_behavior::Obstructor(ai_data);
                                       return std::shared_ptr<robot_behavior::RobotBehavior>(obstructor);
                                     },
                                     false  // we don't want to define a goal here !
                                     )));
  registerStrategy("StrikerAi", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                    ai_data,
                                    [&](double time, double dt) {
                                      robot_behavior::StrikerAi* striker_ai = new robot_behavior::StrikerAi(ai_data);
                                      return std::shared_ptr<robot_behavior::RobotBehavior>(striker_ai);
                                    },
                                    false  // we don't want to define a goal here !
                                    )));
  registerStrategy("PredictFutur", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                       ai_data,
                                       [&](double time, double dt) {
                                         robot_behavior::PredictFutur* p = new robot_behavior::PredictFutur(ai_data);
                                         return std::shared_ptr<robot_behavior::RobotBehavior>(p);
                                       },
                                       false)));

  int velocity = 2.0;
  registerStrategy("test_N_velocity_consign", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                  ai_data,
                                                  [&](double time, double dt) {
                                                    robot_behavior::TestVelocityConsign* pt =
                                                        robot_behavior::TestVelocityConsign::getNMovement(ai_data,
                                                                                                          velocity);
                                                    return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                  },
                                                  false  // we don't want to define a goal here !
                                                  )));
  registerStrategy("test_E_velocity_consign", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                  ai_data,
                                                  [&](double time, double dt) {
                                                    robot_behavior::TestVelocityConsign* pt =
                                                        robot_behavior::TestVelocityConsign::getEMovement(ai_data,
                                                                                                          velocity);
                                                    return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                  },
                                                  false  // we don't want to define a goal here !
                                                  )));
  registerStrategy("test_W_velocity_consign", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                  ai_data,
                                                  [&](double time, double dt) {
                                                    robot_behavior::TestVelocityConsign* pt =
                                                        robot_behavior::TestVelocityConsign::getWMovement(ai_data,
                                                                                                          velocity);
                                                    return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                  },
                                                  false  // we don't want to define a goal here !
                                                  )));
  registerStrategy("test_S_velocity_consign", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                  ai_data,
                                                  [&](double time, double dt) {
                                                    robot_behavior::TestVelocityConsign* pt =
                                                        robot_behavior::TestVelocityConsign::getSMovement(ai_data,
                                                                                                          velocity);
                                                    return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                  },
                                                  false  // we don't want to define a goal here !
                                                  )));
  registerStrategy("test_NW_velocity_consign", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::TestVelocityConsign* pt =
                                                         robot_behavior::TestVelocityConsign::getNWMovement(ai_data,
                                                                                                            velocity);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));
  registerStrategy("test_NE_velocity_consign", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::TestVelocityConsign* pt =
                                                         robot_behavior::TestVelocityConsign::getNEMovement(ai_data,
                                                                                                            velocity);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));
  registerStrategy("test_SW_velocity_consign", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::TestVelocityConsign* pt =
                                                         robot_behavior::TestVelocityConsign::getSWMovement(ai_data,
                                                                                                            velocity);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));
  registerStrategy("test_SE_velocity_consign", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   ai_data,
                                                   [&](double time, double dt) {
                                                     robot_behavior::TestVelocityConsign* pt =
                                                         robot_behavior::TestVelocityConsign::getSEMovement(ai_data,
                                                                                                            velocity);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(pt);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));

  registerStrategy("TestInfra", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                    ai_data,
                                    [&](double time, double dt) {
                                      robot_behavior::TestInfra* p = new robot_behavior::TestInfra(ai_data);
                                      return std::shared_ptr<robot_behavior::RobotBehavior>(p);
                                    },
                                    false)));
  registerStrategy("TestPassDribbler", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                           ai_data,
                                           [&](double time, double dt) {
                                             robot_behavior::PassDribbler* p =
                                                 new robot_behavior::PassDribbler(ai_data);
                                             p->declarePointToPass(allyGoalCenter());
                                             return std::shared_ptr<robot_behavior::RobotBehavior>(p);
                                           },
                                           false)));
  registerStrategy("TestPass", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                   ai_data,
                                   [&](double time, double dt) {
                                     robot_behavior::Pass* p = new robot_behavior::Pass(ai_data);
                                     // p->declare_point_to_pass(ally_goal_center());
                                     p->declareRobotToPass(2, vision::Team::Ally);
                                     return std::shared_ptr<robot_behavior::RobotBehavior>(p);
                                   },
                                   false)));
  registerStrategy("WaitPass", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                   ai_data,
                                   [&](double time, double dt) {
                                     robot_behavior::WaitPass* p = new robot_behavior::WaitPass(ai_data);
                                     return std::shared_ptr<robot_behavior::RobotBehavior>(p);
                                   },
                                   false)));

  registerStrategy("test_angular_only_relative_velocity_consign",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::TestRelativeVelocityConsign* p =
                             robot_behavior::TestRelativeVelocityConsign::getMovementAngularVelocityOnly(ai_data,
                                                                                                         M_PI / 2);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(p);
                       },
                       false  // we don't want to define a goal here !
                       )));

  registerStrategy("test_linear_only_relative_velocity_consign",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::TestRelativeVelocityConsign* p =
                             robot_behavior::TestRelativeVelocityConsign::getMovementLinearVelocityOnly(ai_data,
                                                                                                        Vector2d(0, 1));
                         return std::shared_ptr<robot_behavior::RobotBehavior>(p);
                       },
                       false  // we don't want to define a goal here !
                       )));

  registerStrategy("test_linear_and_angular_relative_velocity_consign",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       ai_data,
                       [&](double time, double dt) {
                         robot_behavior::TestRelativeVelocityConsign* p =
                             robot_behavior::TestRelativeVelocityConsign::getMovementAngularAndLinearVelocity(
                                 ai_data, Vector2d(0, 1), M_PI * 7 / 3);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(p);
                       },
                       false  // we don't want to define a goal here !
                       )));

  registerStrategy(strategy::Halt::name, std::shared_ptr<strategy::Strategy>(new strategy::Halt(ai_data)));
  registerStrategy(strategy::TareAndSynchronize::name,
                   std::shared_ptr<strategy::Strategy>(new strategy::TareAndSynchronize(ai_data)));
  assignStrategy(strategy::Halt::name, 0.0, getTeamIds());  // TODO TIME !
                                                            // strategy_was_assigned = false;
}

void Manual::assignPointOfViewAndGoalie()
{
  // DEBUG(team_color);
  // DEBUG(ai::Team::Yellow);
  changeTeamAndPointOfView(team_color_, goal_to_positive_axis_);
}

void Manual::setTeamColor(ai::Team team_color)
{
  this->team_color_ = team_color;
}

void Manual::defineGoalToPositiveAxis(bool value)
{
  this->goal_to_positive_axis_ = value;
}

void Manual::update(double time)
{
  // if( not( get_strategy_<Strategy::Tare_and_synchronize>().is_tared_and_synchronized() ) ){
  //   assign_strategy( Strategy::Tare_and_synchronize::name, time, get_valid_player_ids() );
  // }
  // update_strategies(time);
  updateCurrentStrategies(time);
  assignPointOfViewAndGoalie();
  // if( ! strategy_was_assigned ){
  //    assign_strategy(
  //        "Goalie",
  //        //"Position Follower",
  //        time, get_team_ids()
  //    );
  //    strategy_was_assigned = true;
  //}
}

Manual::~Manual()
{
}

};  // namespace manager
};  // namespace rhoban_ssl
