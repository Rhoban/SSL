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
  , game_state(game_state)
  , penalty_strats(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , goalie_strats(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , offensive_strats(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , stop_strats(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , halt_strats(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , defensive_strats(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , kick_strats(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
  , kick_strats_indirect(1 + ai::Constants::NB_OF_ROBOTS_BY_TEAM)
{
  penalty_strats[8] = { Strategy::GoalieStrat::name, Strategy::Mur_2::name, Strategy::Defensive2::name, PROTECT_BALL };
  penalty_strats[7] = { Strategy::GoalieStrat::name, Strategy::Mur_2::name, Strategy::Defensive2::name, PROTECT_BALL };
  penalty_strats[6] = { Strategy::GoalieStrat::name, Strategy::Mur_2::name, Strategy::Defensive2::name, PROTECT_BALL };
  penalty_strats[5] = { Strategy::GoalieStrat::name, Strategy::Mur_2::name, Strategy::Defensive::name, PROTECT_BALL };
  penalty_strats[4] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::Defensive::name, PROTECT_BALL };
  penalty_strats[3] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::Defensive::name };
  penalty_strats[2] = { Strategy::GoalieStrat::name, Strategy::Defensive::name };
  penalty_strats[1] = { Strategy::GoalieStrat::name };

  goalie_strats[8] = { Strategy::GoalieStrat::name };
  goalie_strats[7] = { Strategy::GoalieStrat::name };
  goalie_strats[6] = { Strategy::GoalieStrat::name };
  goalie_strats[5] = { Strategy::GoalieStrat::name };
  goalie_strats[4] = { Strategy::GoalieStrat::name };
  goalie_strats[3] = { Strategy::GoalieStrat::name };
  goalie_strats[2] = { Strategy::GoalieStrat::name };
  goalie_strats[1] = { Strategy::GoalieStrat::name };

  kick_strats[8] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name, Strategy::Mur_stop::name,
                     Strategy::Mur_2::name, Strategy::Defensive2::name };
  kick_strats[7] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name, Strategy::Mur_stop::name,
                     Strategy::Mur_2::name, Strategy::Defensive::name };
  kick_strats[6] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name, Strategy::Mur_stop::name,
                     Strategy::Mur_2::name };
  kick_strats[5] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name, Strategy::Mur_stop::name,
                     Strategy::Mur::name };
  kick_strats[4] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name, Strategy::Mur_stop::name };
  kick_strats[3] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name, Strategy::Mur::name };
  kick_strats[2] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name };
  kick_strats[1] = { Strategy::GoalieStrat::name };

  kick_strats_indirect[8] = { Strategy::GoalieStrat::name, Strategy::AttaqueWithSupportMs::name,
                              Strategy::Mur_stop::name, Strategy::Mur_2::name, Strategy::Defensive::name };
  kick_strats_indirect[7] = { Strategy::GoalieStrat::name, Strategy::AttaqueWithSupportMs::name,
                              Strategy::Mur_stop::name, Strategy::Mur::name, Strategy::Defensive::name };
  kick_strats_indirect[6] = { Strategy::GoalieStrat::name, Strategy::AttaqueWithSupportMs::name,
                              Strategy::Mur_stop::name, Strategy::Mur::name };
  kick_strats_indirect[5] = { Strategy::GoalieStrat::name, Strategy::AttaqueWithSupportMs::name,
                              Strategy::Mur_stop::name };
  kick_strats_indirect[4] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name, Strategy::Mur_stop::name };
  kick_strats_indirect[3] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name, Strategy::Mur::name };
  kick_strats_indirect[2] = { Strategy::GoalieStrat::name, Strategy::StrikerKick::name };
  kick_strats_indirect[1] = { Strategy::GoalieStrat::name };

  offensive_strats[8] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::Defensive2::name,
                          Strategy::StrikerV2::name, Strategy::Offensive::name };
  offensive_strats[7] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::Defensive2::name,
                          Strategy::StrikerV2::name, Strategy::Offensive::name };
  offensive_strats[6] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::Defensive2::name,
                          Strategy::StrikerV2::name, Strategy::Offensive::name };
  offensive_strats[5] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::Defensive2::name,
                          Strategy::StrikerV2::name };
  offensive_strats[4] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::Defensive::name,
                          Strategy::StrikerV2::name };
  offensive_strats[3] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::StrikerV2::name };
  offensive_strats[2] = { Strategy::GoalieStrat::name, Strategy::StrikerV2::name };
  offensive_strats[1] = { Strategy::GoalieStrat::name };

  defensive_strats[8] = { Strategy::GoalieStrat::name, Strategy::Mur_2::name, Strategy::Defensive2::name,
                          Strategy::Offensive::name };
  defensive_strats[7] = { Strategy::GoalieStrat::name, Strategy::Mur_2::name, Strategy::Defensive2::name,
                          Strategy::Offensive::name };
  defensive_strats[6] = { Strategy::GoalieStrat::name, Strategy::Mur_2::name, Strategy::Defensive2::name,
                          Strategy::Offensive::name };
  defensive_strats[5] = { Strategy::GoalieStrat::name, Strategy::Mur_2::name, Strategy::Defensive::name,
                          Strategy::StrikerV2::name };
  defensive_strats[4] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::Defensive::name,
                          Strategy::StrikerV2::name };
  defensive_strats[3] = { Strategy::GoalieStrat::name, Strategy::Mur::name, Strategy::StrikerV2::name };
  defensive_strats[2] = { Strategy::GoalieStrat::name, Strategy::Offensive::name };
  defensive_strats[1] = { Strategy::GoalieStrat::name };

  stop_strats[8] = { Strategy::GoalieStrat::name, Strategy::Mur_stop::name, Strategy::Prepare_kickoff::name };
  stop_strats[7] = { Strategy::GoalieStrat::name, Strategy::Mur_stop::name, Strategy::Prepare_kickoff::name };
  stop_strats[6] = { Strategy::GoalieStrat::name, Strategy::Mur_stop::name, Strategy::Prepare_kickoff::name };
  stop_strats[5] = { Strategy::GoalieStrat::name, Strategy::Mur_stop::name, Strategy::Mur_2::name };
  stop_strats[4] = { Strategy::GoalieStrat::name, Strategy::Mur_stop::name, Strategy::Mur::name };
  stop_strats[3] = { Strategy::GoalieStrat::name, Strategy::Mur_stop::name };
  stop_strats[2] = { Strategy::GoalieStrat::name, Strategy::Mur::name };
  stop_strats[1] = { Strategy::GoalieStrat::name };

  halt_strats[8] = { Strategy::Tare_and_synchronize::name, Strategy::Halt::name };
  halt_strats[7] = { Strategy::Tare_and_synchronize::name, Strategy::Halt::name };
  halt_strats[6] = { Strategy::Tare_and_synchronize::name, Strategy::Halt::name };
  halt_strats[5] = { Strategy::Tare_and_synchronize::name, Strategy::Halt::name };
  halt_strats[4] = { Strategy::Tare_and_synchronize::name, Strategy::Halt::name };
  halt_strats[3] = { Strategy::Tare_and_synchronize::name, Strategy::Halt::name };
  halt_strats[2] = { Strategy::Tare_and_synchronize::name, Strategy::Halt::name };
  halt_strats[1] = { Strategy::Tare_and_synchronize::name };

  registerStrategy(Strategy::Halt::name, std::shared_ptr<Strategy::Strategy>(new Strategy::Halt(ai_data)));
  registerStrategy(Strategy::StrikerV2::name, std::shared_ptr<Strategy::Strategy>(new Strategy::StrikerV2(ai_data)));
  registerStrategy(Strategy::Tare_and_synchronize::name,
                    std::shared_ptr<Strategy::Strategy>(new Strategy::Tare_and_synchronize(ai_data)));
  registerStrategy(Strategy::Prepare_kickoff::name,
                    std::shared_ptr<Strategy::Strategy>(new Strategy::Prepare_kickoff(ai_data)));
  registerStrategy(GOALIE, std::shared_ptr<Strategy::Strategy>(new Strategy::From_robot_behavior(
                                ai_data,
                                [&](double time, double dt) {
                                  Robot_behavior::Goalie* goalie = new Robot_behavior::Goalie(ai_data);
                                  return std::shared_ptr<Robot_behavior::RobotBehavior>(goalie);
                                },
                                true)));
  registerStrategy(PROTECT_BALL, std::shared_ptr<Strategy::Strategy>(new Strategy::From_robot_behavior(
                                      ai_data,
                                      [&](double time, double dt) {
                                        Robot_behavior::ProtectBall* protect_ball =
                                            new Robot_behavior::ProtectBall(ai_data);
                                        return std::shared_ptr<Robot_behavior::RobotBehavior>(protect_ball);
                                      },
                                      false)));
  registerStrategy(Strategy::Offensive::name, std::shared_ptr<Strategy::Strategy>(new Strategy::Offensive(ai_data)));
  registerStrategy(Strategy::StrikerKick::name,
                    std::shared_ptr<Strategy::Strategy>(new Strategy::StrikerKick(ai_data)));
  registerStrategy(Strategy::Mur_stop::name, std::shared_ptr<Strategy::Strategy>(new Strategy::Mur_stop(ai_data)));
  registerStrategy(Strategy::Defensive::name, std::shared_ptr<Strategy::Strategy>(new Strategy::Defensive(ai_data)));
  registerStrategy(Strategy::Defensive2::name, std::shared_ptr<Strategy::Strategy>(new Strategy::Defensive2(ai_data)));
  registerStrategy(Strategy::Mur::name, std::shared_ptr<Strategy::Strategy>(new Strategy::Mur(ai_data)));
  registerStrategy(Strategy::Mur_2::name, std::shared_ptr<Strategy::Strategy>(new Strategy::Mur_2(ai_data)));
  registerStrategy(Strategy::Mur_2_passif::name,
                    std::shared_ptr<Strategy::Strategy>(new Strategy::Mur_2_passif(ai_data)));
  registerStrategy(Strategy::AttaqueWithSupportMs::name,
                    std::shared_ptr<Strategy::Strategy>(new Strategy::AttaqueWithSupportMs(ai_data)));
  registerStrategy(Strategy::StrikerWithSupport::name,
                    std::shared_ptr<Strategy::Strategy>(new Strategy::StrikerWithSupport(ai_data)));
  registerStrategy(Strategy::GoalieStrat::name,
                    std::shared_ptr<Strategy::Strategy>(new Strategy::GoalieStrat(ai_data)));
  assignStrategy(Strategy::Halt::name, 0.0,
                  getTeamIds());  // TODO TIME !
}

void PlanVeschambres::start_stop()
{
  setBallAvoidanceForAllRobots(true);
  future_strats = stop_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}

void PlanVeschambres::start_running()
{
  setBallAvoidanceForAllRobots(false);
  if (ballPosition().getX() <= 0)
  {
    future_strats = defensive_strats[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part = true;
  }
  else
  {
    future_strats = offensive_strats[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part = false;
  }
  declareAndAssignNextStrategies(future_strats);
}
void PlanVeschambres::start_halt()
{
  setBallAvoidanceForAllRobots(true);
  future_strats = halt_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}

void PlanVeschambres::start_direct_kick_ally()
{
  setBallAvoidanceForAllRobots(false);
  future_strats = kick_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}
void PlanVeschambres::start_direct_kick_opponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats = defensive_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}

void PlanVeschambres::start_indirect_kick_ally()
{
  setBallAvoidanceForAllRobots(false);
  future_strats = kick_strats_indirect[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}
void PlanVeschambres::start_indirect_kick_opponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats = defensive_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}

void PlanVeschambres::start_prepare_kickoff_ally()
{
  setBallAvoidanceForAllRobots(true);
  future_strats = offensive_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}
void PlanVeschambres::start_prepare_kickoff_opponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats = defensive_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}

void PlanVeschambres::start_kickoff_ally()
{
  setBallAvoidanceForAllRobots(false);
  future_strats = offensive_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}
void PlanVeschambres::start_kickoff_opponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats = defensive_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}

void PlanVeschambres::start_penalty_ally()
{
  setBallAvoidanceForAllRobots(false);
  future_strats = penalty_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}
void PlanVeschambres::start_penalty_opponent()
{
  setBallAvoidanceForAllRobots(true);
  future_strats = stop_strats[Manager::getValidPlayerIds().size() + 1];
  declareAndAssignNextStrategies(future_strats);
}

// Continue

void PlanVeschambres::continue_stop()
{
}

void PlanVeschambres::continue_running()
{
  if (ballPosition().getX() <= 0 and not(ball_was_in_ally_part))
  {
    clearStrategyAssignement();
    future_strats = defensive_strats[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part = true;
    declareAndAssignNextStrategies(future_strats);
  }
  else if (ballPosition().getX() > 0 and ball_was_in_ally_part)
  {
    clearStrategyAssignement();
    future_strats = offensive_strats[Manager::getValidPlayerIds().size() + 1];
    ball_was_in_ally_part = false;
    declareAndAssignNextStrategies(future_strats);
  }
}
void PlanVeschambres::continue_halt()
{
}

void PlanVeschambres::continue_direct_kick_ally()
{
}
void PlanVeschambres::continue_direct_kick_opponent()
{
}

void PlanVeschambres::continue_indirect_kick_ally()
{
}
void PlanVeschambres::continue_indirect_kick_opponent()
{
}

void PlanVeschambres::continue_prepare_kickoff_ally()
{
}
void PlanVeschambres::continue_prepare_kickoff_opponent()
{
}

void PlanVeschambres::continue_kickoff_ally()
{
}
void PlanVeschambres::continue_kickoff_opponent()
{
}

void PlanVeschambres::continue_penalty_ally()
{
}
void PlanVeschambres::continue_penalty_opponent()
{
}

PlanVeschambres::~PlanVeschambres()
{
}

};  // namespace Manager
};  // namespace rhoban_ssl
