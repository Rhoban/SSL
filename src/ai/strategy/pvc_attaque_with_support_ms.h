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

#pragma once

#include "strategy.h"
// #include <robot_behavior/robot_follower.h>
#include <robot_behavior/pvc_striker.h>
#include <robot_behavior/pvc_striker_ai.h>
#include <robot_behavior/pvc_search_shoot_area.h>
//#include <robot_behavior/pass.h>
//#include <robot_behavior/pass_dribbler.h>
#include <robot_behavior/pvc_wait_pass.h>
//#include <robot_behavior/slow_striker.h>
#include <core/machine_state.h>

namespace rhoban_ssl
{
namespace strategy
{
class AttaqueWithSupportMs : public Strategy
{
  // Machine state
  struct state_name
  {
    // name : robot1_robot2
    static const constexpr char* strike_search = "strike_search";
    static const constexpr char* search_strike = "search_strike";

    static const constexpr char* pass_search = "pass_search";
    static const constexpr char* search_pass = "search_pass";

    static const constexpr char* search_waitpass = "search_waitpass";
    static const constexpr char* waitpass_search = "waitpass_search";
  };

  struct edge_name
  {
    // db1 = distance_ball_robot_1
    // db2 = distance_ball_robot_2
    static const constexpr char* db1_sup_db2 = "db1_sup_db2";
    static const constexpr char* db1_inf_db2 = "db1_inf_db2";

    // fgbm = find_goal_best_move
    static const constexpr char* fgbm_score_inf_seuil_1 = "fgbm_score_inf_seuil_1";  // if striker is robot 1
    static const constexpr char* fgbm_score_inf_seuil_2 = "fgbm_score_inf_seuil_2";  // if striker is robot 2

    static const constexpr char* fgbm_score_sup_seuil_1_plus_constante = "fgbm_score_sup_seuil_1_plus_constante";
    static const constexpr char* fgbm_score_sup_seuil_2_plus_constante = "fgbm_score_sup_seuil_2_plus_constante";

    static const constexpr char* infra_1_on = "infra_1_on";
    static const constexpr char* infra_2_on = "infra_2_on";

    // db1 = distance_ball_robot_1
    // db2 = distance_ball_robot_2
    static const constexpr char* db1_inf_seuil_or_time_inf_tempo = "db1_inf_seuil_or_time_inf_tempo";
    static const constexpr char* db2_inf_seuil_or_time_inf_tempo = "db2_inf_seuil_or_time_inf_tempo";

    static const constexpr char* db1_sup_db2_plus_constante = "db1_sup_db2_plus_constante";
    static const constexpr char* db1_plus_constante_inf_db2 = "db1_inf_db2_plus_constante";
  };

  typedef construct_machine_state_infrastructure<std::string, data::AiData, data::AiData>
      machine_state_infrastructure;

private:
  machine_state_infrastructure::MachineState machine_;

  bool behaviors_are_assigned_;
  std::shared_ptr<robot_behavior::StrikerAi> striker_behavior_;
  std::shared_ptr<robot_behavior::SearchShootArea> search_behavior_;
  // std::shared_ptr<Robot_behavior::Pass> pass_behavior;
  // std::shared_ptr<Robot_behavior::Pass_dribbler> pass_behavior;
  // std::shared_ptr<Robot_behavior::SlowStriker> pass_behavior;
  std::shared_ptr<robot_behavior::Striker> pass_behavior_;
  std::shared_ptr<robot_behavior::WaitPass> wait_pass_behavior_;

  double seuil_fgbm_;  // fgbm = find_goal_best_move
  double fgbm_score_;
  double tempo_;

  double begin_time_;
  int ID1_;
  int ID2_;
  double diff_distance_constante_;
  double fgbm_constante_;

  rhoban_geometry::Point robot_1_position_;
  rhoban_geometry::Point robot_2_position_;

public:
  AttaqueWithSupportMs();
  virtual ~AttaqueWithSupportMs();

  virtual int minRobots() const;
  virtual int maxRobots() const;
  virtual GoalieNeed needsGoalie() const;

  static const std::string name;

  virtual void start(double time);
  virtual void stop(double time);

  virtual void update(double time);

  virtual void assignBehaviorToRobots(
      std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt);

  virtual std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
  getStartingPositions(int number_of_avalaible_robots);
  virtual bool getStartingPositionForGoalie(rhoban_geometry::Point& linear_position, ContinuousAngle& angular_position);

  bool isDb1SupDb2();
  bool isDb1InfDb2();

  bool isFgbmScoreInfSeuil_1();
  bool isFgbmScoreInfSeuil_2();

  bool fgbmScoreSupSeuil_1PlusConstante();
  bool fgbmScoreSupSeuil_2PlusConstante();

  bool isInfra_1On();
  bool isInfra_2On();

  bool isDb1InfSeuilOrTimeInfTempo();
  bool isDb2InfSeuilOrTimeInfTempo();

  bool isDb1SupDb2PlusConstante();
  bool isDb1PlusConstanteInfDb2();

  void setSeuilFgbm(double seuil);
};

};  // namespace strategy
};  // namespace rhoban_ssl
