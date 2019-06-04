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

#include "pvc_attaque_with_support_ms.h"
#include "core/print_collection.h"

namespace rhoban_ssl
{
namespace strategy
{
AttaqueWithSupportMs::AttaqueWithSupportMs()
  : Strategy()
  , machine_(Data::get()->ai_data, Data::get()->ai_data)
  , striker_behavior_(std::shared_ptr<robot_behavior::StrikerAi>(new robot_behavior::StrikerAi()))
  , search_behavior_(std::shared_ptr<robot_behavior::SearchShootArea>(new robot_behavior::SearchShootArea()))
  ,
  // pass_behavior(std::shared_ptr<Robot_behavior::Pass>(
  //   new Robot_behavior::Pass(ai_data)
  // )),
  // pass_behavior(std::shared_ptr<Robot_behavior::Pass_dribbler>(
  //   new Robot_behavior::Pass_dribbler(ai_data)
  // )),
  // pass_behavior(std::shared_ptr<Robot_behavior::SlowStriker>(
  //   new Robot_behavior::SlowStriker(ai_data)
  // )),
  pass_behavior_(std::shared_ptr<robot_behavior::Striker>(new robot_behavior::Striker()))
  , wait_pass_behavior_(std::shared_ptr<robot_behavior::WaitPass>(new robot_behavior::WaitPass()))
  , seuil_fgbm_(0.25)
  , fgbm_score_(1)
  , tempo_(2)
  , begin_time_(0)
  , diff_distance_constante_(0.5)
  , fgbm_constante_(0.05)
{ 
  // STATES
  machine_.addState(state_name::strike_search,
                    [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::strike_search);
                    });
  machine_.addState(state_name::search_strike,
                    [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::search_strike);
                    });

  machine_.addState(state_name::pass_search,
                    [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::pass_search);
                    });
  machine_.addState(state_name::search_pass,
                    [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::search_pass);
                    });

  machine_.addState(state_name::search_waitpass,
                    [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::search_waitpass);
                    });
  machine_.addState(state_name::waitpass_search,
                    [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::waitpass_search);
                    });

  // EDGES
  machine_.addEdge(edge_name::db1_sup_db2, state_name::strike_search, state_name::search_strike,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return isDb1SupDb2();
                   });
  machine_.addEdge(edge_name::db1_inf_db2, state_name::search_strike, state_name::strike_search,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return isDb1InfDb2();
                   });

  machine_.addEdge(edge_name::fgbm_score_inf_seuil_1, state_name::strike_search, state_name::pass_search,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return isFgbmScoreInfSeuil_1();
                   });
  machine_.addEdge(edge_name::fgbm_score_inf_seuil_2, state_name::search_strike, state_name::search_pass,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return isFgbmScoreInfSeuil_2();
                   });

  machine_.addEdge(edge_name::fgbm_score_sup_seuil_1_plus_constante, state_name::pass_search, state_name::strike_search,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return fgbmScoreSupSeuil_1PlusConstante();
                   });
  machine_.addEdge(edge_name::fgbm_score_sup_seuil_2_plus_constante, state_name::search_pass, state_name::search_strike,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return fgbmScoreSupSeuil_2PlusConstante();
                   });

  machine_.addEdge(edge_name::infra_1_on, state_name::pass_search, state_name::search_waitpass,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     begin_time_ = time();
                     return isInfra_1On();
                   });
  machine_.addEdge(edge_name::infra_2_on, state_name::search_pass, state_name::waitpass_search,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     begin_time_ = time();
                     return isInfra_2On();
                   });

  machine_.addEdge(edge_name::db2_inf_seuil_or_time_inf_tempo, state_name::search_waitpass, state_name::search_strike,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return isDb2InfSeuilOrTimeInfTempo();
                   });
  machine_.addEdge(edge_name::db1_inf_seuil_or_time_inf_tempo, state_name::waitpass_search, state_name::strike_search,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return isDb1InfSeuilOrTimeInfTempo();
                   });

  machine_.addEdge(edge_name::db1_sup_db2_plus_constante, state_name::pass_search, state_name::search_strike,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return isDb1SupDb2PlusConstante();
                   });
  machine_.addEdge(edge_name::db1_plus_constante_inf_db2, state_name::search_pass, state_name::strike_search,
                   [this](const data::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return isDb1PlusConstanteInfDb2();
                   });

  machine_.exportToFile("/tmp/attaque_with_support_ms.dot");
}

AttaqueWithSupportMs::~AttaqueWithSupportMs()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int AttaqueWithSupportMs::minRobots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int AttaqueWithSupportMs::maxRobots() const
{
  return 2;
}

GoalieNeed AttaqueWithSupportMs::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string AttaqueWithSupportMs::name = "attaque_with_support_ms";

void AttaqueWithSupportMs::start(double time)
{
  DEBUG("START PREPARE KICKOFF");

  ID1_ = playerId(0);
  ID2_ = playerId(1);  // we get the first if in get_player_ids()

  robot_1_position_ = getRobot(ID1_, Ally).getMovement().linearPosition(time);
  robot_2_position_ = getRobot(ID2_, Ally).getMovement().linearPosition(time);

  double db1 = (Vector2d(ballPosition() - robot_1_position_)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position_)).norm();

  if (db1 > db2)
  {
    machine_.addInitState(state_name::strike_search);
  }
  else
  {
    machine_.addInitState(state_name::search_strike);
  }
  machine_.start();

  behaviors_are_assigned_ = false;
}
void AttaqueWithSupportMs::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void AttaqueWithSupportMs::update(double time)
{
}

void AttaqueWithSupportMs::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  // we assign now all the other behavior
  assert(getPlayerIds().size() == 2);

  robot_1_position_ = getRobot(ID1_, Ally).getMovement().linearPosition(time);
  robot_2_position_ = getRobot(ID2_, Ally).getMovement().linearPosition(time);
  fgbm_score_ = findGoalBestMove(ballPosition()).second;

  machine_.run();

  std::string state = *machine_.currentStates().begin();
  // DEBUG(machine.current_states());
  if (state == state_name::strike_search)
  {
    assign_behavior(ID1_, striker_behavior_);
    assign_behavior(ID2_, search_behavior_);
  }
  else if (state == state_name::search_strike)
  {
    assign_behavior(ID1_, search_behavior_);
    assign_behavior(ID2_, striker_behavior_);
  }
  else if (state == state_name::pass_search)
  {
    assign_behavior(ID1_, pass_behavior_);
    // pass_behavior->declare_robot_to_pass( ID2, Vision::Ally );
    pass_behavior_->declarePointToStrike(robot_2_position_);
    assign_behavior(ID2_, search_behavior_);
  }
  else if (state == state_name::search_pass)
  {
    assign_behavior(ID1_, search_behavior_);
    assign_behavior(ID2_, pass_behavior_);
    // pass_behavior->declare_robot_to_pass( ID1, Vision::Ally );
    pass_behavior_->declarePointToStrike(robot_1_position_);
  }
  else if (state == state_name::search_waitpass)
  {
    assign_behavior(ID1_, search_behavior_);
    // pass_behavior->declare_robot_to_pass( ID2, Vision::Ally );
    assign_behavior(ID2_, wait_pass_behavior_);
  }
  else if (state == state_name::waitpass_search)
  {
    assign_behavior(ID1_, wait_pass_behavior_);
    assign_behavior(ID2_, search_behavior_);  // pass_behavior );
    // pass_behavior->declare_robot_to_pass( ID1, Vision::Ally );
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
AttaqueWithSupportMs::getStartingPositions(int number_of_avalaible_robots)
{
  assert(minRobots() <= number_of_avalaible_robots);
  assert(maxRobots() == -1 or number_of_avalaible_robots <= maxRobots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(ballPosition(), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool AttaqueWithSupportMs::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                                        ContinuousAngle& angular_position)
{
  linear_position = Data::get()->field.goalCenter(Ally);
  angular_position = ContinuousAngle(0.0);
  return true;
}

bool AttaqueWithSupportMs::isDb1SupDb2()
{
  double db1 = (Vector2d(ballPosition() - robot_1_position_)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position_)).norm();
  // DEBUG("is_db1_sup_db2 " << db1 << " " << db2);
  // double fgbm_score = find_goal_best_move( ball_position() ).second;
  return (db1 >= db2);  // and (fgbm_score >= seuil_fgbm));
}
bool AttaqueWithSupportMs::isDb1InfDb2()
{
  double db1 = (Vector2d(ballPosition() - robot_1_position_)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position_)).norm();
  // DEBUG("is_db1_inf_db2 " << db1 << " " << db2);
  // double fgbm_score = find_goal_best_move( ball_position() ).second;
  return (db1 < db2);  // and (fgbm_score >= seuil_fgbm));
}

bool AttaqueWithSupportMs::isFgbmScoreInfSeuil_1()
{
  DEBUG(fgbm_score_);
  std::vector<int> vect_obstruct = getRobotInLine(robot_1_position_, robot_2_position_);
  bool free_for_pass = vect_obstruct.empty();
  bool ready_for_pass = search_behavior_->well_positioned;
  return (fgbm_score_ < seuil_fgbm_) and free_for_pass and ready_for_pass;
}
bool AttaqueWithSupportMs::isFgbmScoreInfSeuil_2()
{
  DEBUG(fgbm_score_);
  std::vector<int> vect_obstruct = getRobotInLine(robot_2_position_, robot_1_position_);
  bool free_for_pass = vect_obstruct.empty();
  bool ready_for_pass = search_behavior_->well_positioned;
  return (fgbm_score_ >= seuil_fgbm_ + fgbm_constante_) and free_for_pass and ready_for_pass;
}

bool AttaqueWithSupportMs::fgbmScoreSupSeuil_1PlusConstante()
{
  std::vector<int> vect_obstruct = getRobotInLine(robot_1_position_, robot_2_position_);
  bool free_for_pass = vect_obstruct.empty();
  // bool ready_for_pass = search_behavior->well_positioned;
  return (fgbm_score_ >= seuil_fgbm_ + fgbm_constante_) or not(free_for_pass);
}
bool AttaqueWithSupportMs::fgbmScoreSupSeuil_2PlusConstante()
{
  std::vector<int> vect_obstruct = getRobotInLine(robot_2_position_, robot_1_position_);
  bool free_for_pass = vect_obstruct.empty();
  // bool ready_for_pass = search_behavior->well_positioned;
  return (fgbm_score_ >= seuil_fgbm_ + fgbm_constante_) or not(free_for_pass);
}

bool AttaqueWithSupportMs::isInfra_1On()
{
  return infraRed(ID1_, Ally);
  // double db1 = (Vector2d (ball_position() - robot_1_position)).norm();
  // // DEBUG("DB1 " << db1 );
  // return (db1 < get_robot_radius()+0.1);
}
bool AttaqueWithSupportMs::isInfra_2On()
{
  return infraRed(ID2_, Ally);
  // double db2 = (Vector2d (ball_position() - robot_2_position)).norm();
  // // DEBUG("DB2 " << db2 );
  // return (db2 < get_robot_radius()+0.1);
}

bool AttaqueWithSupportMs::isDb1InfSeuilOrTimeInfTempo()
{
  // return infra_red( ID1, Vision::Ally);
  double db1 = (Vector2d(ballPosition() - robot_1_position_)).norm();
  bool t = (time() - begin_time_ > tempo_);
  return ((db1 < getRobotRadius() + 0.7) || t);
}
bool AttaqueWithSupportMs::isDb2InfSeuilOrTimeInfTempo()
{
  // return infra_red( ID2, Vision::Ally);
  double db2 = (Vector2d(ballPosition() - robot_2_position_)).norm();
  bool t = (time() - begin_time_ > tempo_);
  return ((db2 < getRobotRadius() + 0.7) || t);
}

bool AttaqueWithSupportMs::isDb1SupDb2PlusConstante()
{
  double db1 = (Vector2d(ballPosition() - robot_1_position_)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position_)).norm();
  // DEBUG("is_db1_sup_db2_plus_constante " << db1 << " " << db2);
  return (db1 > db2 + diff_distance_constante_);
  // return false;
}
bool AttaqueWithSupportMs::isDb1PlusConstanteInfDb2()
{
  double db1 = (Vector2d(ballPosition() - robot_1_position_)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position_)).norm();
  // DEBUG("is_db1_sup_db2_plus_constante " << db1 << " " << db2);
  return (db1 + diff_distance_constante_ < db2);
}

void AttaqueWithSupportMs::setSeuilFgbm(double seuil)
{
  seuil_fgbm_ = seuil;
}

}  // namespace strategy
}  // namespace rhoban_ssl
