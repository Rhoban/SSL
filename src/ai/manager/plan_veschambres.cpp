/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#include "plan_veschambres.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/placer.h>
#include <strategy/prepare_kickoff.h>
#include <strategy/from_robot_behavior.h>

#include <strategy/offensive.h>
#include <strategy/defensive.h>
#include <strategy/defensive_2.h>
#include <strategy/mur_stop.h>
#include <strategy/mur.h>
#include <strategy/mur_2.h>
#include <strategy/mur_2_passif.h>
#include <strategy/attaque_with_support.h>
#include <strategy/attaque_with_support_ms.h>
#include <strategy/striker_with_support.h>
#include <strategy/striker_v2.h>
#include <strategy/striker_kick.h>

#include <strategy/goalie_strat.h>

#include <robot_behavior/goalie.h>
#include <robot_behavior/protect_ball.h>

#include <core/collection.h>
#include <core/print_collection.h>

#define GOALIE "goalie"
#define PROTECT_BALL "protect_ball"

namespace rhoban_ssl
{
namespace manager
{
PlanVeschambres::PlanVeschambres(ai::AiData& ai_data, const GameState& game_state)
  : ManagerWithGameState(ai_data, game_state)
  , game_state_(game_state)
  , penalty_strats_(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , goalie_strats_(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , offensive_strats_(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , stop_strats_(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , halt_strats_(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , defensive_strats_(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , kick_strats_(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , kick_strats_indirect_(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
{
  penalty_strats_[8] = { strategy::GoalieStrat::name, strategy::Mur_2::name, strategy::Defensive2::name, PROTECT_BALL };
  penalty_strats_[7] = { strategy::GoalieStrat::name, strategy::Mur_2::name, strategy::Defensive2::name, PROTECT_BALL };
  penalty_strats_[6] = { strategy::GoalieStrat::name, strategy::Mur_2::name, strategy::Defensive2::name, PROTECT_BALL };
  penalty_strats_[5] = { strategy::GoalieStrat::name, strategy::Mur_2::name, strategy::Defensive::name, PROTECT_BALL };
  penalty_strats_[4] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::Defensive::name, PROTECT_BALL };
  penalty_strats_[3] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::Defensive::name };
  penalty_strats_[2] = { strategy::GoalieStrat::name, strategy::Defensive::name };
  penalty_strats_[1] = { strategy::GoalieStrat::name };

  goalie_strats_[8] = { strategy::GoalieStrat::name };
  goalie_strats_[7] = { strategy::GoalieStrat::name };
  goalie_strats_[6] = { strategy::GoalieStrat::name };
  goalie_strats_[5] = { strategy::GoalieStrat::name };
  goalie_strats_[4] = { strategy::GoalieStrat::name };
  goalie_strats_[3] = { strategy::GoalieStrat::name };
  goalie_strats_[2] = { strategy::GoalieStrat::name };
  goalie_strats_[1] = { strategy::GoalieStrat::name };

  kick_strats_[8] = { strategy::GoalieStrat::name, strategy::StrikerKick::name, strategy::MurStop::name,
                     strategy::Mur_2::name, strategy::Defensive2::name };
  kick_strats_[7] = { strategy::GoalieStrat::name, strategy::StrikerKick::name, strategy::MurStop::name,
                     strategy::Mur_2::name, strategy::Defensive::name };
  kick_strats_[6] = { strategy::GoalieStrat::name, strategy::StrikerKick::name, strategy::MurStop::name,
                     strategy::Mur_2::name };
  kick_strats_[5] = { strategy::GoalieStrat::name, strategy::StrikerKick::name, strategy::MurStop::name,
                     strategy::Mur::name };
  kick_strats_[4] = { strategy::GoalieStrat::name, strategy::StrikerKick::name, strategy::MurStop::name };
  kick_strats_[3] = { strategy::GoalieStrat::name, strategy::StrikerKick::name, strategy::Mur::name };
  kick_strats_[2] = { strategy::GoalieStrat::name, strategy::StrikerKick::name };
  kick_strats_[1] = { strategy::GoalieStrat::name };

  kick_strats_indirect_[8] = { strategy::GoalieStrat::name, strategy::AttaqueWithSupportMs::name,
                              strategy::MurStop::name, strategy::Mur_2::name, strategy::Defensive::name };
  kick_strats_indirect_[7] = { strategy::GoalieStrat::name, strategy::AttaqueWithSupportMs::name,
                              strategy::MurStop::name, strategy::Mur::name, strategy::Defensive::name };
  kick_strats_indirect_[6] = { strategy::GoalieStrat::name, strategy::AttaqueWithSupportMs::name,
                              strategy::MurStop::name, strategy::Mur::name };
  kick_strats_indirect_[5] = { strategy::GoalieStrat::name, strategy::AttaqueWithSupportMs::name,
                              strategy::MurStop::name };
  kick_strats_indirect_[4] = { strategy::GoalieStrat::name, strategy::StrikerKick::name, strategy::MurStop::name };
  kick_strats_indirect_[3] = { strategy::GoalieStrat::name, strategy::StrikerKick::name, strategy::Mur::name };
  kick_strats_indirect_[2] = { strategy::GoalieStrat::name, strategy::StrikerKick::name };
  kick_strats_indirect_[1] = { strategy::GoalieStrat::name };

  offensive_strats_[8] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::Defensive2::name,
                          strategy::StrikerV2::name, strategy::Offensive::name };
  offensive_strats_[7] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::Defensive2::name,
                          strategy::StrikerV2::name, strategy::Offensive::name };
  offensive_strats_[6] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::Defensive2::name,
                          strategy::StrikerV2::name, strategy::Offensive::name };
  offensive_strats_[5] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::Defensive2::name,
                          strategy::StrikerV2::name };
  offensive_strats_[4] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::Defensive::name,
                          strategy::StrikerV2::name };
  offensive_strats_[3] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::StrikerV2::name };
  offensive_strats_[2] = { strategy::GoalieStrat::name, strategy::StrikerV2::name };
  offensive_strats_[1] = { strategy::GoalieStrat::name };

  defensive_strats_[8] = { strategy::GoalieStrat::name, strategy::Mur_2::name, strategy::Defensive2::name,
                          strategy::Offensive::name };
  defensive_strats_[7] = { strategy::GoalieStrat::name, strategy::Mur_2::name, strategy::Defensive2::name,
                          strategy::Offensive::name };
  defensive_strats_[6] = { strategy::GoalieStrat::name, strategy::Mur_2::name, strategy::Defensive2::name,
                          strategy::Offensive::name };
  defensive_strats_[5] = { strategy::GoalieStrat::name, strategy::Mur_2::name, strategy::Defensive::name,
                          strategy::StrikerV2::name };
  defensive_strats_[4] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::Defensive::name,
                          strategy::StrikerV2::name };
  defensive_strats_[3] = { strategy::GoalieStrat::name, strategy::Mur::name, strategy::StrikerV2::name };
  defensive_strats_[2] = { strategy::GoalieStrat::name, strategy::Offensive::name };
  defensive_strats_[1] = { strategy::GoalieStrat::name };

  stop_strats_[8] = { strategy::GoalieStrat::name, strategy::MurStop::name, strategy::PrepareKickoff::name };
  stop_strats_[7] = { strategy::GoalieStrat::name, strategy::MurStop::name, strategy::PrepareKickoff::name };
  stop_strats_[6] = { strategy::GoalieStrat::name, strategy::MurStop::name, strategy::PrepareKickoff::name };
  stop_strats_[5] = { strategy::GoalieStrat::name, strategy::MurStop::name, strategy::Mur_2::name };
  stop_strats_[4] = { strategy::GoalieStrat::name, strategy::MurStop::name, strategy::Mur::name };
  stop_strats_[3] = { strategy::GoalieStrat::name, strategy::MurStop::name };
  stop_strats_[2] = { strategy::GoalieStrat::name, strategy::Mur::name };
  stop_strats_[1] = { strategy::GoalieStrat::name };

  halt_strats_[8] = { strategy::TareAndSynchronize::name, strategy::Halt::name };
  halt_strats_[7] = { strategy::TareAndSynchronize::name, strategy::Halt::name };
  halt_strats_[6] = { strategy::TareAndSynchronize::name, strategy::Halt::name };
  halt_strats_[5] = { strategy::TareAndSynchronize::name, strategy::Halt::name };
  halt_strats_[4] = { strategy::TareAndSynchronize::name, strategy::Halt::name };
  halt_strats_[3] = { strategy::TareAndSynchronize::name, strategy::Halt::name };
  halt_strats_[2] = { strategy::TareAndSynchronize::name, strategy::Halt::name };
  halt_strats_[1] = { strategy::TareAndSynchronize::name };

  registerStrategy(strategy::Halt::name, std::shared_ptr<strategy::Strategy>(new strategy::Halt(ai_data)));
  registerStrategy(strategy::StrikerV2::name, std::shared_ptr<strategy::Strategy>(new strategy::StrikerV2(ai_data)));
  registerStrategy(strategy::TareAndSynchronize::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::TareAndSynchronize(ai_data)));
  registerStrategy(strategy::PrepareKickoff::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::PrepareKickoff(ai_data)));
  registerStrategy(GOALIE, std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                ai_data,
                                [&](double time, double dt) {
                                  robot_behavior::Goalie* goalie = new robot_behavior::Goalie(ai_data);
                                  return std::shared_ptr<robot_behavior::RobotBehavior>(goalie);
                                },
                                true)));
  registerStrategy(PROTECT_BALL, std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                      ai_data,
                                      [&](double time, double dt) {
                                        robot_behavior::ProtectBall* protect_ball =
                                            new robot_behavior::ProtectBall(ai_data);
                                        return std::shared_ptr<robot_behavior::RobotBehavior>(protect_ball);
                                      },
                                      false)));
  registerStrategy(strategy::Offensive::name, std::shared_ptr<strategy::Strategy>(new strategy::Offensive(ai_data)));
  registerStrategy(strategy::StrikerKick::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::StrikerKick(ai_data)));
  registerStrategy(strategy::MurStop::name, std::shared_ptr<strategy::Strategy>(new strategy::MurStop(ai_data)));
  registerStrategy(strategy::Defensive::name, std::shared_ptr<strategy::Strategy>(new strategy::Defensive(ai_data)));
  registerStrategy(strategy::Defensive2::name, std::shared_ptr<strategy::Strategy>(new strategy::Defensive2(ai_data)));
  registerStrategy(strategy::Mur::name, std::shared_ptr<strategy::Strategy>(new strategy::Mur(ai_data)));
  registerStrategy(strategy::Mur_2::name, std::shared_ptr<strategy::Strategy>(new strategy::Mur_2(ai_data)));
  registerStrategy(strategy::Mur_2_passif::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::Mur_2_passif(ai_data)));
  registerStrategy(strategy::AttaqueWithSupportMs::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::AttaqueWithSupportMs(ai_data)));
  registerStrategy(strategy::StrikerWithSupport::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::StrikerWithSupport(ai_data)));
  registerStrategy(strategy::GoalieStrat::name,
                    std::shared_ptr<strategy::Strategy>(new strategy::GoalieStrat(ai_data)));
  assignStrategy(strategy::Halt::name, 0.0,
                  getTeamIds());  // TODO TIME !
}

void PlanVeschambres::startStop()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = stop_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void PlanVeschambres::startRunning()
{
  setBallAvoidanceForAllRobots(false);
  if (ballPosition().getX() <= 0)
  {
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
void PlanVeschambres::startHalt()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = halt_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void PlanVeschambres::startDirectKickAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = kick_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void PlanVeschambres::startDirectKickOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void PlanVeschambres::startIndirectKickAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = kick_strats_indirect_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void PlanVeschambres::startIndirectKickOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void PlanVeschambres::startPrepareKickoffAlly()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void PlanVeschambres::startPrepareKickoffOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void PlanVeschambres::startKickoffAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void PlanVeschambres::startKickoffOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

void PlanVeschambres::startPenaltyAlly()
{
  setBallAvoidanceForAllRobots(false);
  future_strats_ = penalty_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}
void PlanVeschambres::startPenaltyOpponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats_ = stop_strats_[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats_);
}

// Continue

void PlanVeschambres::continueStop()
{
}

void PlanVeschambres::continueRunning()
{
  if (ballPosition().getX() <= 0 and not(ball_was_in_ally_part_))
  {
    clearStrategyAssignement();
    future_strats_ = defensive_strats_[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part_ = true;
    declareAndAssignNextStrategies(future_strats_);
  }
  else if (ballPosition().getX() > 0 and ball_was_in_ally_part_)
  {
    clearStrategyAssignement();
    future_strats_ = offensive_strats_[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part_ = false;
    declareAndAssignNextStrategies(future_strats_);
  }
}
void PlanVeschambres::continueHalt()
{
}

void PlanVeschambres::continueDirectKickAlly()
{
}
void PlanVeschambres::continueDirectKickOpponent()
{
}

void PlanVeschambres::continueIndirectKickAlly()
{
}
void PlanVeschambres::continueIndirectKickOpponent()
{
}

void PlanVeschambres::continuePrepareKickoffAlly()
{
}
void PlanVeschambres::continuePrepareKickoffOpponent()
{
}

void PlanVeschambres::continueKickoffAlly()
{
}
void PlanVeschambres::continueKickoffOpponent()
{
}

void PlanVeschambres::continuePenaltyAlly()
{
}
void PlanVeschambres::continuePenaltyOpponent()
{
}

PlanVeschambres::~PlanVeschambres()
{
}

};  // namespace Manager
};  // namespace rhoban_ssl
