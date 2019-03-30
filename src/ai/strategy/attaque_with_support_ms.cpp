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

#include "attaque_with_support_ms.h"
#include "core/print_collection.h"

namespace rhoban_ssl
{
namespace Strategy
{
AttaqueWithSupportMs::AttaqueWithSupportMs(ai::AiData& ai_data)
  : Strategy(ai_data)
  , machine(ai_data, ai_data)
  , striker_behavior(std::shared_ptr<Robot_behavior::StrikerAi>(new Robot_behavior::StrikerAi(ai_data)))
  , search_behavior(std::shared_ptr<Robot_behavior::SearchShootArea>(new Robot_behavior::SearchShootArea(ai_data)))
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
  pass_behavior(std::shared_ptr<Robot_behavior::Striker>(new Robot_behavior::Striker(ai_data)))
  , wait_pass_behavior(std::shared_ptr<Robot_behavior::WaitPass>(new Robot_behavior::WaitPass(ai_data)))
  , seuil_fgbm(0.25)
  , fgbm_score(1)
  , tempo(2)
  , begin_time(0)
  , diff_distance_constante(0.5)
  , fgbm_constante(0.05)
{
  // STATES
  machine.addState(state_name::strike_search,
                    [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::strike_search);
                    });
  machine.addState(state_name::search_strike,
                    [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::search_strike);
                    });

  machine.addState(state_name::pass_search,
                    [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::pass_search);
                    });
  machine.addState(state_name::search_pass,
                    [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::search_pass);
                    });

  machine.addState(state_name::search_waitpass,
                    [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::search_waitpass);
                    });
  machine.addState(state_name::waitpass_search,
                    [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      // DEBUG(state_name::waitpass_search);
                    });

  // EDGES
  machine.addEdge(edge_name::db1_sup_db2, state_name::strike_search, state_name::search_strike,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return is_db1_sup_db2();
                   });
  machine.addEdge(edge_name::db1_inf_db2, state_name::search_strike, state_name::strike_search,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return is_db1_inf_db2();
                   });

  machine.addEdge(edge_name::fgbm_score_inf_seuil_1, state_name::strike_search, state_name::pass_search,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return is_fgbm_score_inf_seuil_1();
                   });
  machine.addEdge(edge_name::fgbm_score_inf_seuil_2, state_name::search_strike, state_name::search_pass,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return is_fgbm_score_inf_seuil_2();
                   });

  machine.addEdge(edge_name::fgbm_score_sup_seuil_1_plus_constante, state_name::pass_search, state_name::strike_search,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return fgbm_score_sup_seuil_1_plus_constante();
                   });
  machine.addEdge(edge_name::fgbm_score_sup_seuil_2_plus_constante, state_name::search_pass, state_name::search_strike,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return fgbm_score_sup_seuil_2_plus_constante();
                   });

  machine.addEdge(edge_name::infra_1_on, state_name::pass_search, state_name::search_waitpass,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     begin_time = time();
                     return is_infra_1_on();
                   });
  machine.addEdge(edge_name::infra_2_on, state_name::search_pass, state_name::waitpass_search,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     begin_time = time();
                     return is_infra_2_on();
                   });

  machine.addEdge(edge_name::db2_inf_seuil_or_time_inf_tempo, state_name::search_waitpass, state_name::search_strike,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return is_db2_inf_seuil_or_time_inf_tempo();
                   });
  machine.addEdge(edge_name::db1_inf_seuil_or_time_inf_tempo, state_name::waitpass_search, state_name::strike_search,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return is_db1_inf_seuil_or_time_inf_tempo();
                   });

  machine.addEdge(edge_name::db1_sup_db2_plus_constante, state_name::pass_search, state_name::search_strike,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return is_db1_sup_db2_plus_constante();
                   });
  machine.addEdge(edge_name::db1_plus_constante_inf_db2, state_name::search_pass, state_name::strike_search,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     return is_db1_plus_constante_inf_db2();
                   });

  machine.exportToFile("/tmp/attaque_with_support_ms.dot");
}

AttaqueWithSupportMs::~AttaqueWithSupportMs()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int AttaqueWithSupportMs::min_robots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int AttaqueWithSupportMs::max_robots() const
{
  return 2;
}

Goalie_need AttaqueWithSupportMs::needs_goalie() const
{
  return Goalie_need::NO;
}

const std::string AttaqueWithSupportMs::name = "attaque_with_support_ms";

void AttaqueWithSupportMs::start(double time)
{
  DEBUG("START PREPARE KICKOFF");

  ID1 = player_id(0);
  ID2 = player_id(1);  // we get the first if in get_player_ids()

  robot_1_position = getRobot(ID1, vision::Team::Ally).getMovement().linearPosition(time);
  robot_2_position = getRobot(ID2, vision::Team::Ally).getMovement().linearPosition(time);

  double db1 = (Vector2d(ballPosition() - robot_1_position)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position)).norm();

  if (db1 > db2)
  {
    machine.addInitState(state_name::strike_search);
  }
  else
  {
    machine.addInitState(state_name::search_strike);
  }
  machine.start();

  behaviors_are_assigned = false;
}
void AttaqueWithSupportMs::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void AttaqueWithSupportMs::update(double time)
{
}

void AttaqueWithSupportMs::assign_behavior_to_robots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  // we assign now all the other behavior
  assert(get_player_ids().size() == 2);

  robot_1_position = getRobot(ID1, vision::Team::Ally).getMovement().linearPosition(time);
  robot_2_position = getRobot(ID2, vision::Team::Ally).getMovement().linearPosition(time);
  fgbm_score = findGoalBestMove(ballPosition()).second;

  machine.run();

  std::string state = *machine.currentStates().begin();
  // DEBUG(machine.current_states());
  if (state == state_name::strike_search)
  {
    assign_behavior(ID1, striker_behavior);
    assign_behavior(ID2, search_behavior);
  }
  else if (state == state_name::search_strike)
  {
    assign_behavior(ID1, search_behavior);
    assign_behavior(ID2, striker_behavior);
  }
  else if (state == state_name::pass_search)
  {
    assign_behavior(ID1, pass_behavior);
    // pass_behavior->declare_robot_to_pass( ID2, Vision::Team::Ally );
    pass_behavior->declare_point_to_strik(robot_2_position);
    assign_behavior(ID2, search_behavior);
  }
  else if (state == state_name::search_pass)
  {
    assign_behavior(ID1, search_behavior);
    assign_behavior(ID2, pass_behavior);
    // pass_behavior->declare_robot_to_pass( ID1, Vision::Team::Ally );
    pass_behavior->declare_point_to_strik(robot_1_position);
  }
  else if (state == state_name::search_waitpass)
  {
    assign_behavior(ID1, search_behavior);
    // pass_behavior->declare_robot_to_pass( ID2, Vision::Team::Ally );
    assign_behavior(ID2, wait_pass_behavior);
  }
  else if (state == state_name::waitpass_search)
  {
    assign_behavior(ID1, wait_pass_behavior);
    assign_behavior(ID2, search_behavior);  // pass_behavior );
    // pass_behavior->declare_robot_to_pass( ID1, Vision::Team::Ally );
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
AttaqueWithSupportMs::get_starting_positions(int number_of_avalaible_robots)
{
  assert(min_robots() <= number_of_avalaible_robots);
  assert(max_robots() == -1 or number_of_avalaible_robots <= max_robots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(ballPosition(), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool AttaqueWithSupportMs::get_starting_position_for_goalie(rhoban_geometry::Point& linear_position,
                                                            ContinuousAngle& angular_position)
{
  linear_position = allyGoalCenter();
  angular_position = ContinuousAngle(0.0);
  return true;
}

bool AttaqueWithSupportMs::is_db1_sup_db2()
{
  double db1 = (Vector2d(ballPosition() - robot_1_position)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position)).norm();
  // DEBUG("is_db1_sup_db2 " << db1 << " " << db2);
  // double fgbm_score = find_goal_best_move( ball_position() ).second;
  return (db1 >= db2);  // and (fgbm_score >= seuil_fgbm));
}
bool AttaqueWithSupportMs::is_db1_inf_db2()
{
  double db1 = (Vector2d(ballPosition() - robot_1_position)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position)).norm();
  // DEBUG("is_db1_inf_db2 " << db1 << " " << db2);
  // double fgbm_score = find_goal_best_move( ball_position() ).second;
  return (db1 < db2);  // and (fgbm_score >= seuil_fgbm));
}

bool AttaqueWithSupportMs::is_fgbm_score_inf_seuil_1()
{
  DEBUG(fgbm_score);
  std::vector<int> vect_obstruct = getRobotInLine(robot_1_position, robot_2_position);
  bool free_for_pass = vect_obstruct.empty();
  bool ready_for_pass = search_behavior->well_positioned;
  return (fgbm_score < seuil_fgbm) and free_for_pass and ready_for_pass;
}
bool AttaqueWithSupportMs::is_fgbm_score_inf_seuil_2()
{
  DEBUG(fgbm_score);
  std::vector<int> vect_obstruct = getRobotInLine(robot_2_position, robot_1_position);
  bool free_for_pass = vect_obstruct.empty();
  bool ready_for_pass = search_behavior->well_positioned;
  return (fgbm_score >= seuil_fgbm + fgbm_constante) and free_for_pass and ready_for_pass;
}

bool AttaqueWithSupportMs::fgbm_score_sup_seuil_1_plus_constante()
{
  std::vector<int> vect_obstruct = getRobotInLine(robot_1_position, robot_2_position);
  bool free_for_pass = vect_obstruct.empty();
  // bool ready_for_pass = search_behavior->well_positioned;
  return (fgbm_score >= seuil_fgbm + fgbm_constante) or not(free_for_pass);
}
bool AttaqueWithSupportMs::fgbm_score_sup_seuil_2_plus_constante()
{
  std::vector<int> vect_obstruct = getRobotInLine(robot_2_position, robot_1_position);
  bool free_for_pass = vect_obstruct.empty();
  // bool ready_for_pass = search_behavior->well_positioned;
  return (fgbm_score >= seuil_fgbm + fgbm_constante) or not(free_for_pass);
}

bool AttaqueWithSupportMs::is_infra_1_on()
{
  return infraRed(ID1, vision::Team::Ally);
  // double db1 = (Vector2d (ball_position() - robot_1_position)).norm();
  // // DEBUG("DB1 " << db1 );
  // return (db1 < get_robot_radius()+0.1);
}
bool AttaqueWithSupportMs::is_infra_2_on()
{
  return infraRed(ID2, vision::Team::Ally);
  // double db2 = (Vector2d (ball_position() - robot_2_position)).norm();
  // // DEBUG("DB2 " << db2 );
  // return (db2 < get_robot_radius()+0.1);
}

bool AttaqueWithSupportMs::is_db1_inf_seuil_or_time_inf_tempo()
{
  // return infra_red( ID1, Vision::Team::Ally);
  double db1 = (Vector2d(ballPosition() - robot_1_position)).norm();
  bool t = (time() - begin_time > tempo);
  return ((db1 < getRobotRadius() + 0.7) || t);
}
bool AttaqueWithSupportMs::is_db2_inf_seuil_or_time_inf_tempo()
{
  // return infra_red( ID2, Vision::Team::Ally);
  double db2 = (Vector2d(ballPosition() - robot_2_position)).norm();
  bool t = (time() - begin_time > tempo);
  return ((db2 < getRobotRadius() + 0.7) || t);
}

bool AttaqueWithSupportMs::is_db1_sup_db2_plus_constante()
{
  double db1 = (Vector2d(ballPosition() - robot_1_position)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position)).norm();
  // DEBUG("is_db1_sup_db2_plus_constante " << db1 << " " << db2);
  return (db1 > db2 + diff_distance_constante);
  // return false;
}
bool AttaqueWithSupportMs::is_db1_plus_constante_inf_db2()
{
  double db1 = (Vector2d(ballPosition() - robot_1_position)).norm();
  double db2 = (Vector2d(ballPosition() - robot_2_position)).norm();
  // DEBUG("is_db1_sup_db2_plus_constante " << db1 << " " << db2);
  return (db1 + diff_distance_constante < db2);
}

void AttaqueWithSupportMs::set_seuil_fgbm(double seuil)
{
  seuil_fgbm = seuil;
}

}  // namespace Strategy
}  // namespace rhoban_ssl
