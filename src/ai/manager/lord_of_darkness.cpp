/*
    This file is part of SSL.

    GNAGNAGNA

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

// Strategies
#include "lord_of_darkness.h"

//#include "game_informations.h"

#include <strategy/halt.h>
#include <strategy/keeper/keeper_strat.h>
#include <strategy/wall_2.h>
#include <strategy/wall_2_passif.h>
#include <strategy/wall.h>
#include <strategy/striker_v2.h>
#include <strategy/offensive.h>
#include <strategy/defensive.h>
#include <strategy/defensive_2.h>
#include <robot_behavior/go_to_xy.h>
#include <robot_behavior/protect_ball.h>
#include <robot_behavior/stop_not_far.h>
#include <robot_behavior/stop_not_far_2.h>
#include <robot_behavior/stop_not_far_3.h>
#include <robot_behavior/striker_ai.h>
#include <strategy/from_robot_behavior.h>
#include <strategy/mur_stop.h>
#include <strategy/prepare_kickoff.h>
#include <strategy/striker_kick.h>
#include <strategy/attackms.h>
#include <robot_behavior/wall_stop_2.h>
#include <robot_behavior/wall_stop.h>

#include <data.h>

namespace rhoban_ssl
{
namespace manager
{
LordOfDarkness::LordOfDarkness(std::string name)
  : ManagerWithGameState(name)
  , offensive_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , defensive_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , halt_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , kickoff_ally_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , kickoff_opponent_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , penalty_strats_a_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , penalty_strats_o_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , goalie_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , stop_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , kick_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , kick_strats_indirect_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
  , direct_opponent_strats_(1 + ai::Config::NB_OF_ROBOTS_BY_TEAM)
{
  // strategies arrays begin at 1(case 0 unused) to directly acces the good strategy by giving number of disponible

  offensive_strats_[8] = { strategy::KeeperStrat::name, strategy::Wall::name, strategy::Defensive2::name,
                           strategy::StrikerV2::name, strategy::Offensive::name };
  offensive_strats_[7] = { strategy::KeeperStrat::name, strategy::Wall::name, strategy::Defensive2::name,
                           strategy::StrikerV2::name, strategy::Offensive::name };
  offensive_strats_[6] = { strategy::KeeperStrat::name, strategy::Wall::name, strategy::Defensive2::name,
                           strategy::StrikerV2::name, strategy::Offensive::name };
  offensive_strats_[5] = { strategy::KeeperStrat::name, strategy::Wall::name, strategy::Defensive2::name,
                           strategy::StrikerV2::name };
  offensive_strats_[4] = { strategy::KeeperStrat::name, strategy::Wall::name, strategy::Defensive::name,
                           strategy::StrikerV2::name };
  offensive_strats_[3] = { strategy::KeeperStrat::name, strategy::Wall::name, strategy::StrikerV2::name };
  offensive_strats_[2] = { strategy::KeeperStrat::name, strategy::StrikerV2::name };
  offensive_strats_[1] = { strategy::KeeperStrat::name };

  defensive_strats_[8] = { strategy::KeeperStrat::name, strategy::Wall_2::name, strategy::Defensive2::name,
                           strategy::Offensive::name };
  defensive_strats_[7] = { strategy::KeeperStrat::name, strategy::Wall_2::name, strategy::Defensive2::name,
                           strategy::Offensive::name };
  defensive_strats_[6] = { strategy::KeeperStrat::name, strategy::Wall_2::name, strategy::Defensive2::name,
                           strategy::Offensive::name };
  defensive_strats_[5] = { strategy::KeeperStrat::name, strategy::Wall_2::name, strategy::Defensive::name,
                           strategy::StrikerV2::name };
  defensive_strats_[4] = { strategy::KeeperStrat::name, strategy::Wall::name, strategy::Defensive::name,
                           strategy::StrikerV2::name };
  defensive_strats_[3] = { strategy::KeeperStrat::name, strategy::Wall::name, strategy::StrikerV2::name };
  defensive_strats_[2] = { strategy::KeeperStrat::name, strategy::Offensive::name };
  defensive_strats_[1] = { strategy::KeeperStrat::name };

  // halt_strats
  halt_strats_[8] = { strategy::Halt::name };
  halt_strats_[7] = { strategy::Halt::name };
  halt_strats_[6] = { strategy::Halt::name };
  halt_strats_[5] = { strategy::Halt::name };
  halt_strats_[4] = { strategy::Halt::name };
  halt_strats_[3] = { strategy::Halt::name };
  halt_strats_[2] = { strategy::Halt::name };
  halt_strats_[1] = { strategy::Halt::name };
  /*
    stop_strats_[8] = { strategy::KeeperStrat::name, strategy::MurStop::name, strategy::PrepareKickoff::name };
    stop_strats_[7] = { strategy::KeeperStrat::name, strategy::MurStop::name, strategy::PrepareKickoff::name };
    stop_strats_[6] = { strategy::KeeperStrat::name, strategy::MurStop::name, strategy::PrepareKickoff::name };
    stop_strats_[5] = { strategy::KeeperStrat::name, strategy::MurStop::name, strategy::Wall_2::name };
    stop_strats_[4] = { strategy::KeeperStrat::name, strategy::MurStop::name, strategy::Wall::name };
    stop_strats_[3] = { strategy::KeeperStrat::name, strategy::MurStop::name };
    stop_strats_[2] = { strategy::KeeperStrat::name, strategy::Wall::name };
    stop_strats_[1] = { strategy::KeeperStrat::name };*/

  // kickoff_ally
  kickoff_ally_strats_[8] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_ally_placement_M",
                              "kickoff_ally_placement_R", "kickoff_ally_placement_L" };
  kickoff_ally_strats_[7] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_ally_placement_M",
                              "kickoff_ally_placement_R", "kickoff_ally_placement_L" };
  kickoff_ally_strats_[6] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_ally_placement_M",
                              "kickoff_ally_placement_R", "kickoff_ally_placement_L" };
  kickoff_ally_strats_[5] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_ally_placement_M",
                              "kickoff_ally_placement_R" };
  kickoff_ally_strats_[4] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_ally_placement_M" };
  kickoff_ally_strats_[3] = { strategy::KeeperStrat::name, strategy::Wall::name, "kickoff_ally_placement_M" };
  kickoff_ally_strats_[2] = { strategy::KeeperStrat::name, "kickoff_ally_placement_M" };
  kickoff_ally_strats_[1] = { strategy::KeeperStrat::name };

  // kickoff_opponent
  kickoff_opponent_strats_[8] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_opponent_placement_M",
                                  "kickoff_ally_placement_R", "kickoff_ally_placement_L" };
  kickoff_opponent_strats_[7] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_opponent_placement_M",
                                  "kickoff_ally_placement_R", "kickoff_ally_placement_L" };
  kickoff_opponent_strats_[6] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_opponent_placement_M",
                                  "kickoff_ally_placement_R", "kickoff_ally_placement_L" };
  kickoff_opponent_strats_[5] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_opponent_placement_M",
                                  "kickoff_ally_placement_R" };
  kickoff_opponent_strats_[4] = { strategy::KeeperStrat::name, strategy::Wall_2::name, "kickoff_opponent_placement_M" };
  kickoff_opponent_strats_[3] = { strategy::KeeperStrat::name, strategy::Wall::name, "kickoff_opponent_placement_M" };
  kickoff_opponent_strats_[2] = { strategy::KeeperStrat::name, "kickoff_opponent_placement_M" };
  kickoff_opponent_strats_[1] = { strategy::KeeperStrat::name };

  penalty_strats_a_[8] = { strategy::KeeperStrat::name, "GT1", "GT2", "GT3", "GT4", "GT" };
  penalty_strats_a_[7] = { strategy::KeeperStrat::name, "GT1", "GT2", "GT3", "GT4", "GT" };
  penalty_strats_a_[6] = { strategy::KeeperStrat::name, "GT1", "GT2", "GT3", "GT4", "GT" };
  penalty_strats_a_[5] = { strategy::KeeperStrat::name, "GT1", "GT2", "GT3", "GT" };
  penalty_strats_a_[4] = { strategy::KeeperStrat::name, "GT1", "GT2", "GT" };
  penalty_strats_a_[3] = { strategy::KeeperStrat::name, "GT1", "GT" };
  penalty_strats_a_[2] = { strategy::KeeperStrat::name, "GT" };
  penalty_strats_a_[1] = { strategy::KeeperStrat::name };

  penalty_strats_o_[8] = { "GTG", "GT1", "GT2", "GT3", "GT4", "GT5" };
  penalty_strats_o_[7] = { "GTG", "GT1", "GT2", "GT3", "GT4", "GT5" };
  penalty_strats_o_[6] = { "GTG", "GT1", "GT2", "GT3", "GT4", "GT5" };
  penalty_strats_o_[5] = { "GTG", "GT1", "GT2", "GT3", "GT4" };
  penalty_strats_o_[4] = { "GTG", "GT1", "GT2", "GT3" };
  penalty_strats_o_[3] = { "GTG", "GT1", "GT2" };
  penalty_strats_o_[2] = { "GTG", "GT1" };
  penalty_strats_o_[1] = { "GTG" };

  goalie_strats_[8] = { strategy::KeeperStrat::name };
  goalie_strats_[7] = { strategy::KeeperStrat::name };
  goalie_strats_[6] = { strategy::KeeperStrat::name };
  goalie_strats_[5] = { strategy::KeeperStrat::name };
  goalie_strats_[4] = { strategy::KeeperStrat::name };
  goalie_strats_[3] = { strategy::KeeperStrat::name };
  goalie_strats_[2] = { strategy::KeeperStrat::name };
  goalie_strats_[1] = { strategy::KeeperStrat::name };

  kick_strats_[8] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name,
                      strategy::Wall_2::name, strategy::Defensive2::name };
  kick_strats_[7] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name,
                      strategy::Wall_2::name, strategy::Defensive::name };
  kick_strats_[6] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name,
                      strategy::Wall_2::name };
  kick_strats_[5] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name,
                      strategy::Wall::name };
  kick_strats_[4] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name };
  kick_strats_[3] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::Wall::name };
  kick_strats_[2] = { strategy::KeeperStrat::name, strategy::StrikerKick::name };
  kick_strats_[1] = { strategy::KeeperStrat::name };

  kick_strats_indirect_[8] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name,
                               strategy::Wall_2::name, strategy::Defensive::name };
  kick_strats_indirect_[7] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name,
                               strategy::Wall::name, strategy::Defensive::name };
  kick_strats_indirect_[6] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name,
                               strategy::Wall::name };
  kick_strats_indirect_[5] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name };
  kick_strats_indirect_[4] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::MurStop::name };
  kick_strats_indirect_[3] = { strategy::KeeperStrat::name, strategy::StrikerV2::name, strategy::Wall::name };
  kick_strats_indirect_[2] = { strategy::KeeperStrat::name, strategy::StrikerV2::name };
  kick_strats_indirect_[1] = { strategy::KeeperStrat::name };

  direct_opponent_strats_[8] = { strategy::KeeperStrat::name, "WS1", "WS2", "SNF1", "SNF2", "SNF3" };
  direct_opponent_strats_[7] = { strategy::KeeperStrat::name, "WS1", "WS2", "SNF1", "SNF2", "SNF3" };
  direct_opponent_strats_[6] = { strategy::KeeperStrat::name, "WS1", "WS2", "SNF1", "SNF2", "SNF3" };
  direct_opponent_strats_[5] = { strategy::KeeperStrat::name, "WS1", "WS2", "SNF1", "SNF2" };
  direct_opponent_strats_[4] = { strategy::KeeperStrat::name, "WS1", "SNF1", "SNF2" };
  direct_opponent_strats_[3] = { strategy::KeeperStrat::name, "WS1", "SNF1" };
  direct_opponent_strats_[2] = { strategy::KeeperStrat::name, "SNF1" };
  direct_opponent_strats_[1] = { strategy::KeeperStrat::name };

  // Register strategy.
  registerStrategy(strategy::Halt::name, std::shared_ptr<strategy::Strategy>(new strategy::Halt()));
  registerStrategy(strategy::KeeperStrat::name, std::shared_ptr<strategy::Strategy>(new strategy::KeeperStrat()));
  registerStrategy(strategy::Wall_2::name, std::shared_ptr<strategy::Strategy>(new strategy::Wall_2()));
  registerStrategy(strategy::Wall2Passif::name, std::shared_ptr<strategy::Strategy>(new strategy::Wall2Passif()));
  registerStrategy(strategy::Wall::name, std::shared_ptr<strategy::Strategy>(new strategy::Wall()));
  registerStrategy(strategy::Defensive2::name, std::shared_ptr<strategy::Strategy>(new strategy::Defensive2()));
  registerStrategy(strategy::Defensive::name, std::shared_ptr<strategy::Strategy>(new strategy::Defensive()));
  registerStrategy(strategy::StrikerV2::name, std::shared_ptr<strategy::Strategy>(new strategy::StrikerV2()));
  registerStrategy(strategy::Offensive::name, std::shared_ptr<strategy::Strategy>(new strategy::Offensive()));
  registerStrategy(strategy::StrikerKick::name, std::shared_ptr<strategy::Strategy>(new strategy::StrikerKick()));
  registerStrategy(strategy::AttaqueWithSupportMs::name,
                   std::shared_ptr<strategy::Strategy>(new strategy::AttaqueWithSupportMs()));

  registerStrategy("kickoff_ally_placement_M", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   [&](double time, double dt) {
                                                     robot_behavior::GoToXY* go = new robot_behavior::GoToXY(
                                                         rhoban_geometry::Point(-0.35, 0), 0.01, true);

                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));
  registerStrategy("kickoff_opponent_placement_M", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                       [&](double time, double dt) {
                                                         robot_behavior::GoToXY* go = new robot_behavior::GoToXY(
                                                             rhoban_geometry::Point(-0.6, 0), 0.01, true);
                                                         return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                                                       },
                                                       false  // we don't want to define a goal here !
                                                       )));

  registerStrategy("kickoff_ally_placement_L", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   [&](double time, double dt) {
                                                     robot_behavior::GoToXY* go = new robot_behavior::GoToXY(
                                                         rhoban_geometry::Point(-2, 2), 0.01, true);
                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));

  registerStrategy("kickoff_ally_placement_R", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                                   [&](double time, double dt) {
                                                     robot_behavior::GoToXY* go = new robot_behavior::GoToXY(
                                                         rhoban_geometry::Point(-2, -2), 0.01, true);

                                                     return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                                                   },
                                                   false  // we don't want to define a goal here !
                                                   )));

  registerStrategy("SNF1", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                               [&](double time, double dt) {
                                 robot_behavior::StopNotFar* go = new robot_behavior::StopNotFar();
                                 return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                               },
                               false  // we don't want to define a goal here !
                               )));
  registerStrategy("SNF2", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                               [&](double time, double dt) {
                                 robot_behavior::StopNotFar2* go = new robot_behavior::StopNotFar2();
                                 return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                               },
                               false  // we don't want to define a goal here !
                               )));
  registerStrategy("SNF3", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                               [&](double time, double dt) {
                                 robot_behavior::StopNotFar3* go = new robot_behavior::StopNotFar3();
                                 return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                               },
                               false  // we don't want to define a goal here !
                               )));
  registerStrategy("GT", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                             [&](double time, double dt) {
                               robot_behavior::GoToXY* go =
                                   new robot_behavior::GoToXY(rhoban_geometry::Point(3.3, 0), 0.01, true);
                               return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                             },
                             false  // we don't want to define a goal here !
                             )));
  registerStrategy("GTG", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                              [&](double time, double dt) {
                                robot_behavior::GoToXY* go =
                                    new robot_behavior::GoToXY(Data::get()->field.goalCenter(Ally), 0.01, true);
                                return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                              },
                              true)));

  registerStrategy("WS2", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                              [&](double time, double dt) {
                                robot_behavior::WallStop2* go = new robot_behavior::WallStop2();
                                return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                              },
                              false)));
  registerStrategy("WS1", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                              [&](double time, double dt) {
                                robot_behavior::WallStop1* go = new robot_behavior::WallStop1();
                                return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                              },
                              false)));

  registerStrategy(strategy::PrepareKickoff::name, std::shared_ptr<strategy::Strategy>(new strategy::PrepareKickoff()));
  registerStrategy(strategy::MurStop::name, std::shared_ptr<strategy::Strategy>(new strategy::MurStop()));

  registerStrategy(PROTECT_BALL, std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                     [&](double time, double dt) {
                                       robot_behavior::ProtectBall* protect_ball = new robot_behavior::ProtectBall();
                                       return std::shared_ptr<robot_behavior::RobotBehavior>(protect_ball);
                                     },
                                     false)));
  registerStrategy("GT1", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                              [&](double time, double dt) {
                                robot_behavior::GoToXY* go =
                                    new robot_behavior::GoToXY(rhoban_geometry::Point(-1, 2.8), 0.01, true);
                                return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                              },
                              false  // we don't want to define a goal here !
                              )));
  registerStrategy("GT2", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                              [&](double time, double dt) {
                                robot_behavior::GoToXY* go =
                                    new robot_behavior::GoToXY(rhoban_geometry::Point(-1, -2.8), 0.01, true);
                                return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                              },
                              false  // we don't want to define a goal here !
                              )));
  registerStrategy("GT3", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                              [&](double time, double dt) {
                                robot_behavior::GoToXY* go =
                                    new robot_behavior::GoToXY(rhoban_geometry::Point(-1, 1.2), 0.01, true);
                                return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                              },
                              false  // we don't want to define a goal here !
                              )));

  registerStrategy("GT4", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                              [&](double time, double dt) {
                                robot_behavior::GoToXY* go =
                                    new robot_behavior::GoToXY(rhoban_geometry::Point(-1, -1.2), 0.01, true);
                                return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                              },
                              false  // we don't want to define a goal here !
                              )));
  registerStrategy("GT5", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                              [&](double time, double dt) {
                                robot_behavior::GoToXY* go =
                                    new robot_behavior::GoToXY(rhoban_geometry::Point(-1, 0), 0.01, true);
                                return std::shared_ptr<robot_behavior::RobotBehavior>(go);
                              },
                              false  // we don't want to define a goal here !
                              )));
}

void LordOfDarkness::startStop()
{
  DEBUG("START STOP");
  setBallAvoidanceForAllRobots(true);
  future_strats_ = direct_opponent_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void LordOfDarkness::startRunning()
{
  DEBUG("START RUNNING");
  setBallAvoidanceForAllRobots(false);
  if (Data::get()->ball.movement_sample[0].linear_position.x <= -2)
  {
    //+ 1 because the method getValidPlayerIds() doesn't count goalie.
    future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part_ = true;
  }
  else
  {
    future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part_ = false;
  }
  declareAndAssignNextStrategies(future_strats_);
}
void LordOfDarkness::startHalt()
{
  DEBUG("START halt");
  // setBallAvoidanceForAllRobots(true);
  future_strats_ = halt_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void LordOfDarkness::startDirectKickAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = kick_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void LordOfDarkness::startDirectKickOpponent()  //
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = direct_opponent_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void LordOfDarkness::startIndirectKickAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = kick_strats_indirect_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void LordOfDarkness::startIndirectKickOpponent()  //
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = direct_opponent_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void LordOfDarkness::startPrepareKickoffAlly()
{
  // setBallAvoidanceForAllRobots(true);
  future_strats_ = kickoff_ally_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void LordOfDarkness::startPrepareKickoffOpponent()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = kickoff_opponent_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void LordOfDarkness::startKickoffAlly()
{
  // setBallAvoidanceForAllRobots(false);
  future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void LordOfDarkness::startKickoffOpponent()
{
  setBallAvoidanceForAllRobots(false);
  // TODO: Mettre une strat ici ?
  // nope pas besoin
}

void LordOfDarkness::startPreparePenaltyAlly()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = penalty_strats_a_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void LordOfDarkness::startPreparePenaltyOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = penalty_strats_o_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void LordOfDarkness::startPenaltyAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void LordOfDarkness::startPenaltyOpponent()
{
  setBallAvoidanceForAllRobots(true);
}

// Continue

void LordOfDarkness::continueStop()
{
}

void LordOfDarkness::continueRunning()
{
  if (Data::get()->ball.movement_sample[0].linear_position.x <= -2 and not(ball_was_in_ally_part_))
  {
    clearStrategyAssignement();
    future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part_ = true;
    declareAndAssignNextStrategies(future_strats_);
  }
  else if (Data::get()->ball.movement_sample[0].linear_position.x > -2 and ball_was_in_ally_part_)
  {
    clearStrategyAssignement();
    future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part_ = false;
    declareAndAssignNextStrategies(future_strats_);
  }
}
void LordOfDarkness::continueHalt()
{
}

void LordOfDarkness::continueDirectKickAlly()
{
}
void LordOfDarkness::continueDirectKickOpponent()
{
}

void LordOfDarkness::continueIndirectKickAlly()
{
}
void LordOfDarkness::continueIndirectKickOpponent()
{
}

void LordOfDarkness::continuePrepareKickoffAlly()
{
}
void LordOfDarkness::continuePrepareKickoffOpponent()
{
}

void LordOfDarkness::continueKickoffAlly()
{
}
void LordOfDarkness::continueKickoffOpponent()
{
}

void LordOfDarkness::continuePreparePenaltyAlly()
{
}
void LordOfDarkness::continuePreparePenaltyOpponent()
{
}

void LordOfDarkness::continuePenaltyAlly()
{
}
void LordOfDarkness::continuePenaltyOpponent()
{
}

LordOfDarkness::~LordOfDarkness()
{
}

};  // namespace manager
};  // namespace rhoban_ssl
