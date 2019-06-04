/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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
#include <manager/manual.h>

#include <strategy/from_robot_behavior.h>
#include <robot_behavior/position_follower.h>
#include <robot_behavior/tutorials/beginner/goto_ball.h>
#include <robot_behavior/tutorials/beginner/go_corner.h>
#include <robot_behavior/tutorials/beginner/goalie.h>
#include <robot_behavior/tutorials/medium/defender.h>
#include <robot_behavior/tutorials/beginner/see_ball.h>
#include <robot_behavior/tutorials/beginner/see_robot.h>
#include <robot_behavior/tutorials/beginner/robot_near_ball.h>
#include <robot_behavior/tutorials/beginner/robot_have_ball.h>
#include <robot_behavior/tutorials/beginner/annotations_ball_position.h>
#include <robot_behavior/pvc_degageur.h>
#include <robot_behavior/pvc_goalie.h>
#include <robot_behavior/pvc_mur_def_kick.h>
#include <robot_behavior/pvc_mur_defensor.h>
#include <robot_behavior/pvc_obstructor.h>
#include <robot_behavior/pvc_robot_follower.h>
#include <robot_behavior/pvc_search_shoot_area.h>
#include <robot_behavior/pvc_slow_striker.h>
#include <robot_behavior/pvc_striker_ai.h>
#include <robot_behavior/pvc_striker.h>
#include <robot_behavior/pvc_wait_pass.h>

#include <strategy/pvc_attaque_with_support_ms.h>
#include <strategy/pvc_defensive_2.h>
#include <strategy/pvc_defensive.h>
#include <strategy/pvc_goalie_strat.h>
#include <strategy/pvc_mur_2_passif.h>
#include <strategy/pvc_mur_2.h>
#include <strategy/pvc_mur_stop.h>

namespace rhoban_ssl
{
namespace manager
{
Manual::Manual(std::string name) : Manager(name)
{
  registerStrategy("Beginner go to ball",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       [&](double time, double dt) {
                         robot_behavior::Beginner::Goto_ball* beginner_goto_ball =
                             new robot_behavior::Beginner::Goto_ball();
                         return std::shared_ptr<robot_behavior::RobotBehavior>(beginner_goto_ball);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("Beginner - Goalie", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                            [&](double time, double dt) {
                                              robot_behavior::beginner::Goalie* goalie =
                                                  new robot_behavior::beginner::Goalie();
                                              return std::shared_ptr<robot_behavior::RobotBehavior>(goalie);
                                            },
                                            false  // we don't want to define a goal here !
                                            )));
  registerStrategy("Medium - Defender", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                            [&](double time, double dt) {
                                              robot_behavior::medium::Defender* defender =
                                                  new robot_behavior::medium::Defender();
                                              return std::shared_ptr<robot_behavior::RobotBehavior>(defender);
                                            },
                                            false  // we don't want to define a goal here !
                                            )));
  registerStrategy("Beginner Annotations - Ball position",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       [&](double time, double dt) {
                         robot_behavior::BeginnerAnnotationsBallPosition* ball_position =
                             new robot_behavior::BeginnerAnnotationsBallPosition();
                         return std::shared_ptr<robot_behavior::RobotBehavior>(ball_position);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("Beginner - See ball", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                              [&](double time, double dt) {
                                                robot_behavior::beginner::SeeBall* see_ball =
                                                    new robot_behavior::beginner::SeeBall();
                                                return std::shared_ptr<robot_behavior::RobotBehavior>(see_ball);
                                              },
                                              false  // we don't want to define a goal here !
                                              )));
  registerStrategy("Beginner - See Robot 3", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                 [&](double time, double dt) {
                                                   robot_behavior::Beginner::SeeRobot* see_robot =
                                                       new robot_behavior::Beginner::SeeRobot();
                                                   see_robot->setRobotIdToSee(3);
                                                   return std::shared_ptr<robot_behavior::RobotBehavior>(see_robot);
                                                 },
                                                 false  // we don't want to define a goal here !
                                                 )));
  registerStrategy("Beginner - Robot near ball", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                     [&](double time, double dt) {
                                                       robot_behavior::BeginnerRobotNearBall* near_ball =
                                                           new robot_behavior::BeginnerRobotNearBall();
                                                       return std::shared_ptr<robot_behavior::RobotBehavior>(near_ball);
                                                     },
                                                     false  // we don't want to define a goal here !
                                                     )));
  registerStrategy("Beginner - Robot have ball", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                     [&](double time, double dt) {
                                                       robot_behavior::BeginnerRobotHaveBall* have_ball =
                                                           new robot_behavior::BeginnerRobotHaveBall();
                                                       return std::shared_ptr<robot_behavior::RobotBehavior>(have_ball);
                                                     },
                                                     false  // we don't want to define a goal here !
                                                     )));

  registerStrategy("PVC - B - Degageur", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                             [&](double time, double dt) {
                                               robot_behavior::Degageur* deg = new robot_behavior::Degageur();
                                               return std::shared_ptr<robot_behavior::RobotBehavior>(deg);
                                             },
                                             false  // we don't want to define a goal here !
                                             )));
  registerStrategy("PVC - B - Goalie", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                           [&](double time, double dt) {
                                             robot_behavior::Goalie* goal = new robot_behavior::Goalie();
                                             return std::shared_ptr<robot_behavior::RobotBehavior>(goal);
                                           },
                                           false  // decoration
                                           )));
  registerStrategy("PVC - B - Mur Def Kick", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                 [&](double time, double dt) {
                                                   robot_behavior::MurDefKick* mur = new robot_behavior::MurDefKick();
                                                   return std::shared_ptr<robot_behavior::RobotBehavior>(mur);
                                                 },
                                                 false  // we don't want to define a goal here !
                                                 )));
  registerStrategy("PVC - B - Mur Defensor", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                 [&](double time, double dt) {
                                                   robot_behavior::MurDefensor* mur = new robot_behavior::MurDefensor();
                                                   return std::shared_ptr<robot_behavior::RobotBehavior>(mur);
                                                 },
                                                 false  // we don't want to define a goal here !
                                                 )));
  registerStrategy("PVC - B - Obstructor", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                               [&](double time, double dt) {
                                                 robot_behavior::Obstructor* obs = new robot_behavior::Obstructor();
                                                 obs->declareRobotToObstruct(2, Opponent);
                                                 return std::shared_ptr<robot_behavior::RobotBehavior>(obs);
                                               },
                                               false  // we don't want to define a goal here !
                                               )));
  registerStrategy("PVC - B - Robot Follower", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   [&](double time, double dt) {
                                                     robot_behavior::RobotFollower* follower =
                                                         new robot_behavior::RobotFollower();
                                                     follower->declareRobotToFollow(0, Vector2d(1, 0.5), Ally);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(follower);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));
  registerStrategy("PVC - B - Search Shoot Area", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                      [&](double time, double dt) {
                                                        robot_behavior::SearchShootArea* searcher =
                                                            new robot_behavior::SearchShootArea();
                                                        return std::shared_ptr<robot_behavior::RobotBehavior>(searcher);
                                                      },
                                                      false  // we don't want to define a goal here !
                                                      )));
  registerStrategy("PVC - B - Slow Striker",
                   std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                       [&](double time, double dt) {
                         robot_behavior::SlowStriker* striker = new robot_behavior::SlowStriker();
                         striker->declarePointToStrike(Data::get()->field.goalCenter(Opponent));
                         striker->declareRobotToPass(1, Ally);
                         return std::shared_ptr<robot_behavior::RobotBehavior>(striker);
                       },
                       false  // we don't want to define a goal here !
                       )));
  registerStrategy("PVC - B - Striker AI", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                               [&](double time, double dt) {
                                                 robot_behavior::StrikerAi* striker = new robot_behavior::StrikerAi();
                                                 return std::shared_ptr<robot_behavior::RobotBehavior>(striker);
                                               },
                                               false  // we don't want to define a goal here !
                                               )));
  registerStrategy("PVC - B - Striker", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                            [&](double time, double dt) {
                                              robot_behavior::Striker* striker = new robot_behavior::Striker();
                                              return std::shared_ptr<robot_behavior::RobotBehavior>(striker);
                                            },
                                            false  // we don't want to define a goal here !
                                            )));
  registerStrategy("PVC - B - Wait pass", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                              [&](double time, double dt) {
                                                robot_behavior::WaitPass* wait = new robot_behavior::WaitPass();
                                                return std::shared_ptr<robot_behavior::RobotBehavior>(wait);
                                              },
                                              false  // we don't want to define a goal here !
                                              )));

  registerStrategy("PVC - Attaque With Support Ms",
                   std::shared_ptr<strategy::Strategy>(new strategy::AttaqueWithSupportMs()));
  registerStrategy("PVC - Defensive 2", std::shared_ptr<strategy::Strategy>(new strategy::Defensive2()));
  registerStrategy("PVC - Defensive 1", std::shared_ptr<strategy::Strategy>(new strategy::Defensive()));
  registerStrategy("PVC - Goalie Strat (need goalie)",
                   std::shared_ptr<strategy::Strategy>(new strategy::GoalieStrat()));
  registerStrategy("PVC - Mur 2 Passif", std::shared_ptr<strategy::Strategy>(new strategy::Mur_2_passif()));
  registerStrategy("PVC - Mur 2", std::shared_ptr<strategy::Strategy>(new strategy::Mur_2()));
  registerStrategy("PVC - Mur Stop", std::shared_ptr<strategy::Strategy>(new strategy::MurStop()));
}

void Manual::update()
{
  updateCurrentStrategies();
}

Json::Value Manual::getProperties()
{
  Json::Value properties;
  properties_factory.addSetValue("name_test", "");

  properties = properties_factory.getJson();

  properties_factory.clear();
  return properties;
}

void Manual::setProperties(Json::Value json)
{
  DEBUG(json);
}

Manual::~Manual()
{
}
}  // namespace manager
}  // namespace rhoban_ssl
