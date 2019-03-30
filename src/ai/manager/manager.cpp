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

#include "manager.h"

#include <debug.h>
#include <strategy/halt.h>
#include <algorithm>
#include <core/collection.h>
#include <core/print_collection.h>
#include <algorithm>
#include <math/matching.h>

namespace rhoban_ssl
{
namespace Manager
{
int Manager::get_goalie_opponent_id() const
{
  return goalie_opponent_id;
}

void Manager::declare_goalie_opponent_id(int goalie_opponent_id)
{
  if (goalie_opponent_id >= ai::Constants::NB_OF_ROBOTS_BY_TEAM)
    return;
  if (this->goalie_opponent_id >= 0)
  {
    ai_data.robots[vision::Opponent][this->goalie_opponent_id].is_goalie = false;
  }
  this->goalie_opponent_id = goalie_opponent_id;
  if (goalie_opponent_id >= 0)
  {
    ai_data.robots[vision::Opponent][goalie_opponent_id].is_goalie = true;
  }
}
void Manager::declare_goalie_id(int goalie_id)
{
  if (goalie_id >= ai::Constants::NB_OF_ROBOTS_BY_TEAM)
    return;
  if (this->goalie_id >= 0)
  {
    ai_data.robots[vision::Ally][this->goalie_id].is_goalie = false;
  }
  this->goalie_id = goalie_id;
  if (goalie_id >= 0)
  {
    ai_data.robots[vision::Ally][this->goalie_id].is_goalie = true;
  }
}
int Manager::get_goalie_id() const
{
  return goalie_id;
}
void Manager::declare_team_ids(const std::vector<int>& team_ids)
{
  this->team_ids = team_ids;
}

ai::Team Manager::get_team() const
{
  return ai_data.team_color;
}
const std::string& Manager::get_team_name() const
{
  return ai_data.team_name;
}

const std::vector<int>& Manager::get_team_ids() const
{
  return team_ids;
}

void Manager::register_strategy(const std::string& strategy_name, std::shared_ptr<Strategy::Strategy> strategy)
{
  assert(strategies.find(strategy_name) == strategies.end());
  strategies[strategy_name] = strategy;
}

void Manager::clear_strategy_assignement()
{
  for (const std::string& name : current_strategy_names)
  {
    get_strategy(name).stop(time());
  }
  current_strategy_names.clear();
  assign_strategy(MANAGER__REMOVE_ROBOTS, time(), get_invalid_team_ids());
}

void Manager::assign_strategy(const std::string& strategy_name, double time, const std::vector<int>& robot_ids,
                              bool assign_goalie)
{
  assert(strategies.find(strategy_name) != strategies.end());  // The name of the strategy is not declared. Please
                                                               // register them with register_strategy() (during the
                                                               // initialisation of your manager for example).
  assert(not(assign_goalie) or
         (assign_goalie and std::find(robot_ids.begin(), robot_ids.end(), goalie_id) ==
                                robot_ids.end()));  // If you declare that you are assigning a goal, you should not
                                                    // declar the goal id inside the list of field robots.

  current_strategy_names.push_front(strategy_name);
  Strategy::Strategy& strategy = get_strategy(strategy_name);

  if (not(static_cast<unsigned int>(strategy.min_robots()) <= robot_ids.size()))
  {
    DEBUG("We have not enough robot for the strategy : "
          << strategy_name << ". Number of robot requested : " << static_cast<unsigned int>(strategy.min_robots())
          << ", number or avalaible robot : " << robot_ids.size() << ".");
  }
  assert(static_cast<unsigned int>(strategy.min_robots()) <= robot_ids.size());

  strategy.set_goalie(goalie_id, assign_goalie);
  strategy.set_goalie_opponent(goalie_opponent_id);
  strategy.set_robot_affectation(robot_ids);
  strategy.start(time);

  std::cout << "Manager: we assign '" << strategy_name << "' with " << robot_ids.size()
            << " field robots : " << strategy.get_player_ids() << "."
            << " Goalie id is " << strategy.get_goalie() << " and is " << (assign_goalie ? "" : "not ")
            << "assigned to the strategy as goalie." << std::endl;
}

Strategy::Strategy& Manager::get_strategy(const std::string& strategy_name)
{
  assert(strategies.find(strategy_name) != strategies.end());
  return *(strategies.at(strategy_name));
}

const Strategy::Strategy& Manager::get_strategy(const std::string& strategy_name) const
{
  assert(strategies.find(strategy_name) != strategies.end());
  return *(strategies.at(strategy_name));
}

const std::list<std::string>& Manager::get_current_strategy_names() const
{
  return current_strategy_names;
}

void Manager::update_strategies(double time)
{
  for (std::pair<std::string, std::shared_ptr<Strategy::Strategy> > elem : strategies)
  {
    elem.second->update(time);
  }
}

void Manager::update_current_strategies(double time)
{
  for (const std::string& name : current_strategy_names)
  {
    get_strategy(name).update(time);
  }
}

void Manager::assign_behavior_to_robots(std::map<int, std::shared_ptr<Robot_behavior::RobotBehavior> >& robot_behaviors,
                                        double time, double dt)
{
  for (const std::string& name : current_strategy_names)
  {
    get_strategy(name).assign_behavior_to_robots(
        [&](int id, std::shared_ptr<Robot_behavior::RobotBehavior> behavior) {
#ifndef NDEBUG
          bool id_is_present = false;
          for (int robot_id : this->get_strategy(name).get_player_ids())
          {
            if (robot_id == id)
            {
              id_is_present = true;
              break;
            }
          }
          if (this->get_strategy(name).have_to_manage_the_goalie() and get_goalie_id() == id)
          {
            id_is_present = true;
          }
          assert(id_is_present);
#endif
          if (id == -1)
            return;
          robot_behaviors[id] = behavior;
          return;
        },
        time, dt);
  }
}

void Manager::change_ally_and_opponent_goalie_id(int blue_goalie_id, int yellow_goalie_id)
{
  declare_goalie_id((get_team() == ai::Team::Yellow) ? yellow_goalie_id : blue_goalie_id);
  declare_goalie_opponent_id((get_team() == ai::Team::Yellow) ? blue_goalie_id : yellow_goalie_id);
}

void Manager::change_team_and_point_of_view(ai::Team team, bool blue_have_it_s_goal_on_positive_x_axis)
{
  if (team != ai::Unknown and get_team() != team)
  {
    assert(team == ai::Blue or team == ai::Yellow);
    ai_data.changeTeamColor(team);
    blueIsNotSet = true;
  }
  // We change the point of view of the team
  if (blueIsNotSet or blueTeamOnPositiveHalf != blue_have_it_s_goal_on_positive_x_axis)
  {
    blueIsNotSet = false;
    blueTeamOnPositiveHalf = blue_have_it_s_goal_on_positive_x_axis;
    if ((get_team() == ai::Blue and blue_have_it_s_goal_on_positive_x_axis) or
        (get_team() == ai::Yellow and !blue_have_it_s_goal_on_positive_x_axis))
    {
      ai_data.changeFrameForAllObjects(rhoban_geometry::Point(0.0, 0.0), Vector2d(-1.0, 0.0), Vector2d(0.0, -1.0));
    }
    else
    {
      ai_data.changeFrameForAllObjects(rhoban_geometry::Point(0.0, 0.0), Vector2d(1.0, 0.0), Vector2d(0.0, 1.0));
    }
  }
}

Manager::Manager(ai::AiData& ai_data)
  : GameInformations(ai_data), blueIsNotSet(true), goalie_id(-1), goalie_opponent_id(-1), ai_data(ai_data)
{
  register_strategy(MANAGER__REMOVE_ROBOTS, std::shared_ptr<Strategy::Strategy>(new Strategy::Halt(ai_data)));
  register_strategy(MANAGER__PLACER, std::shared_ptr<Strategy::Strategy>(new Strategy::Placer(ai_data)));
}

int Manager::time() const
{
  return ai_data.time;
}

int Manager::dt() const
{
  return ai_data.dt;
}

void Manager::affect_invalid_robots_to_invalid_robots_strategy()
{
  Strategy::Strategy& strategy = get_strategy(MANAGER__REMOVE_ROBOTS);
  strategy.stop(time());
  strategy.set_robot_affectation(get_invalid_team_ids());
  strategy.start(time());
}

void Manager::remove_invalid_robots()
{
  detect_invalid_robots();
  affect_invalid_robots_to_invalid_robots_strategy();
}

void Manager::detect_invalid_robots()
{
  // TODO : we need to detect when the list of invalid robot change.
  // When it change, then, we need to reaffect robot ids.

  int nb_valid = 0;
  int n_robots = team_ids.size();
  for (int i = 0; i < n_robots; i++)
  {
    if (ai_data.robotIsValid(team_ids[i]))
    {
      nb_valid++;
    }
  }
  valid_team_ids.clear();
  valid_player_ids.clear();

  invalid_team_ids.clear();
  for (int i = 0; i < n_robots; i++)
  {
    int id = team_ids[i];
    if (ai_data.robotIsValid(id))
    {
      valid_team_ids.push_back(id);
      if (goalie_id != id)
      {
        valid_player_ids.push_back(id);
      }
    }
    else
    {
      invalid_team_ids.push_back(id);
    }
  }
}

const std::vector<int>& Manager::get_valid_team_ids() const
{
  return valid_team_ids;
}
const std::vector<int>& Manager::get_valid_player_ids() const
{
  return valid_player_ids;
}
const std::vector<int>& Manager::get_invalid_team_ids() const
{
  return invalid_team_ids;
}
Manager::~Manager()
{
}

std::vector<std::string> Manager::get_available_strategies()
{
  std::vector<std::string> strategyNames;

  for (auto& entry : strategies)
  {
    strategyNames.push_back(entry.first);
  }

  return strategyNames;
}

std::list<std::string>
Manager::determine_the_robot_needs_for_the_strategies(const std::list<std::string>& next_strategies)
{
  std::list<std::string> next_valid_strategies;
  minimal_nb_of_robots_to_be_affected = 0;
  strategy_with_arbitrary_number_of_robot = "";
  robot_affectations_by_strategy.clear();
  goal_has_to_be_placed = false;
  for (const std::string& strategy_name : next_strategies)
  {
    // for players
    const Strategy::Strategy& strategy = get_strategy(strategy_name);
    int minimal_number_of_robot = strategy.min_robots();
    if (minimal_number_of_robot + minimal_nb_of_robots_to_be_affected > get_valid_player_ids().size())
    {
      DEBUG("Strategy : " << strategy_name << " need too many robots. We remove them to the list of next strategies.");
      continue;
    }
    else
    {
      next_valid_strategies.push_back(strategy_name);
    }
    minimal_nb_of_robots_to_be_affected += minimal_number_of_robot;
    if (strategy.max_robots() == -1)
    {
      // We want the last one, so we remove previous discovered strategy name
      strategy_with_arbitrary_number_of_robot = strategy_name;
    }
    robot_affectations_by_strategy[strategy_name] = std::vector<int>(minimal_number_of_robot);

    // for goalie
    bool strategy_needs_a_goal = (strategy.needs_goalie() == Strategy::Goalie_need::YES) or
                                 ((get_strategy(strategy_name).needs_goalie() == Strategy::Goalie_need::IF_POSSIBLE) and
                                  (goal_has_to_be_placed));
    if (strategy_needs_a_goal)
    {
      if (goal_has_to_be_placed)
      {
        DEBUG("In strategy : " << strategy_name << ", a goalie is yet defined but the strategy : " << strategy_with_goal
                               << " is also assigned and have to contain a goal too.");
      }
      assert(not(goal_has_to_be_placed));  // Two goal is defined, check you are not assigning two stratgies with a goal
                                           // !
      goal_has_to_be_placed = true;
      strategy_with_goal = strategy_name;
    }
  }
  if (get_valid_player_ids().size() < minimal_nb_of_robots_to_be_affected)
  {
    nb_of_extra_robots = 0;
  }
  else
  {
    nb_of_extra_robots = (get_valid_player_ids().size() - minimal_nb_of_robots_to_be_affected);
  }
  if (strategy_with_arbitrary_number_of_robot != "")
  {
    robot_affectations_by_strategy[strategy_with_arbitrary_number_of_robot].resize(
        robot_affectations_by_strategy[strategy_with_arbitrary_number_of_robot].size() + nb_of_extra_robots);
  }
  /*
      if( strategy_with_arbitrary_number_of_robot != "" ){
          nb_of_extra_robots_non_affected = nb_of_extra_robots;
      }else{
          nb_of_extra_robots_non_affected = 0;
      }
  */
  return next_valid_strategies;
}

void Manager::aggregate_all_starting_position_of_all_strategies(const std::list<std::string>& next_strategies)
{
  starting_positions.clear();
  repartitions_of_starting_positions_in_the_list.clear();

  // For the players
  for (const std::string& strategy_name : next_strategies)
  {
    const Strategy::Strategy& strategy = get_strategy(strategy_name);
    std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> > startings =
        strategy.get_starting_positions(robot_affectations_by_strategy.at(strategy_name).size());
    starting_positions.insert(starting_positions.end(), startings.begin(), startings.end());
    repartitions_of_starting_positions_in_the_list.push_back(
        std::pair<std::string, int>(strategy_name, startings.size()));
  }

  // For the goalie
  if (goal_has_to_be_placed)
  {
    if (!get_strategy(strategy_with_goal)
             .get_starting_position_for_goalie(this->goalie_linear_position, this->goalie_angular_position))
    {
      this->goalie_linear_position = rhoban_geometry::Point(-ai_data.field.fieldLength / 2.0, 0.0);
      this->goalie_angular_position = ContinuousAngle(0.0);
    }
  }
}

void Manager::sort_robot_ordered_by_the_distance_with_starting_position()
{
  assert(starting_positions.size() <= get_valid_player_ids().size());
  robot_consigns = std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> >(get_valid_player_ids().size());
  robot_affectations.resize(get_valid_player_ids().size());

  std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> > choising_positions = list2vector(starting_positions);

  std::function<double(const int& robot_id, const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos)>
      robot_ranking = [this](const int& robot_id, const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos) {
        return Vector2d(pos.first - this->getRobot(robot_id).getMovement().linear_position(time())).normSquare();
      };

  std::function<double(const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos, const int& robot_id)>
      distance_ranking = [this](const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos, const int& robot_id) {
        return Vector2d(pos.first - this->getRobot(robot_id).getMovement().linear_position(time())).normSquare();
      };

  matching::Matchings matchings = matching::galeShapleyAlgorithm(get_valid_player_ids(), choising_positions,
                                                                   robot_ranking, distance_ranking, false, false);

  std::list<int> not_choosen_robot;
  for (unsigned int id : matchings.unaffected_man)
  {
    not_choosen_robot.push_back(get_valid_player_ids()[id]);
  }

  for (unsigned int i = 0; i < choising_positions.size(); i++)
  {
    const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos = choising_positions[i];
    robot_consigns[i] = pos;
    robot_affectations[i] = get_valid_player_ids()[matchings.women_to_man_matchings.at(i)];
  }

  // We place the other robot outsde the field.
  std::list<int>::const_iterator it = not_choosen_robot.begin();
  for (unsigned int i = starting_positions.size(); i < get_valid_player_ids().size(); i++)
  {
    robot_consigns[i] = std::pair<rhoban_geometry::Point, ContinuousAngle>(
        rhoban_geometry::Point(-((5.0 * ai_data.constants.robot_radius) * (i - starting_positions.size()) +
                                 1.5 * ai_data.constants.robot_radius),
                               -ai_data.field.fieldWidth / 2.0 + ai_data.constants.robot_radius),
        ContinuousAngle(0.0));
    robot_affectations[i] = *it;
    it++;
  }
}

void Manager::compute_robot_affectations_to_strategies()
{
  unsigned int cpt_robot = 0;
  unsigned int cpt_extra_robot = 0;
  for (const std::pair<std::string, int>& elem : repartitions_of_starting_positions_in_the_list)
  {
    const std::string& strategy_name = elem.first;
    const int nb_robots = elem.second;
    for (int i = 0; i < nb_robots; i++)
    {
      robot_affectations_by_strategy[strategy_name][i] = robot_affectations.at(cpt_robot + i);
    }

    int nb_robots_with_no_startings = (robot_affectations_by_strategy.at(strategy_name).size() - nb_robots);

    for (int i = 0; i < nb_robots_with_no_startings; i++)
    {
      robot_affectations_by_strategy[strategy_name][nb_robots + i] =
          robot_affectations.at(starting_positions.size() + cpt_extra_robot + i);
    }
    cpt_robot += nb_robots;
    cpt_extra_robot += nb_robots_with_no_startings;
  }
}

void Manager::declare_robot_positions_in_the_placer()
{
  if (goal_has_to_be_placed)
  {
    get_strategy_<Strategy::Placer>(MANAGER__PLACER)
        .set_goalie_positions(goalie_linear_position, goalie_angular_position);
  }
  // else{
  // TODO : should we declare in the strategy that goalie have to be ignored ?
  //}

  assert(starting_positions.size() <= get_valid_team_ids().size());

  get_strategy_<Strategy::Placer>(MANAGER__PLACER).set_positions(robot_affectations, robot_consigns);
}

void Manager::place_all_the_robots(double time, const std::list<std::string>& next_strategies)
{
  declare_next_strategies(next_strategies);
  declare_robot_positions_in_the_placer();
  assign_strategy(MANAGER__PLACER, time, get_valid_player_ids(), goal_has_to_be_placed);
}

const std::vector<int>& Manager::get_robot_affectations(const std::string& strategy_name) const
{
  assert(robot_affectations_by_strategy.find(strategy_name) != robot_affectations_by_strategy.end());
  return robot_affectations_by_strategy.at(strategy_name);
}

void Manager::declare_next_strategies(const std::list<std::string>& next_strategies)
{
  DEBUG("");
  DEBUG("#########################");
  DEBUG("#########################");
  DEBUG("DECLARE NEXT STRATEGIES");
  DEBUG("#########################");
  DEBUG("#########################");
  DEBUG("next_strategies : " << next_strategies);

  std::list<std::string> next_valid_strategies = determine_the_robot_needs_for_the_strategies(next_strategies);

  DEBUG("============== determ. ======");
  DEBUG("nb_of_extra_robots_non_affected : " << nb_of_extra_robots_non_affected);
  DEBUG("minimal_nb_of_robots_to_be_affected : " << minimal_nb_of_robots_to_be_affected);
  DEBUG("nb_of_extra_robots : " << nb_of_extra_robots);
  DEBUG("strategy_with_arbitrary_number_of_robot : " << strategy_with_arbitrary_number_of_robot);
  DEBUG("goal_has_to_be_placed : " << goal_has_to_be_placed);
  DEBUG("strategy_with_goal : " << strategy_with_goal);
  DEBUG("robot_affectations_by_strategy : " << robot_affectations_by_strategy);

  DEBUG("============== aggre. ======");
  aggregate_all_starting_position_of_all_strategies(next_valid_strategies);

  DEBUG("starting_positions : " << starting_positions);
  DEBUG("repartitions_of_starting_positions_in_the_list : " << repartitions_of_starting_positions_in_the_list);

  DEBUG("goalie_linear_position : " << goalie_linear_position);
  DEBUG("goalie_angular_position : " << goalie_angular_position);

  DEBUG("============== sort. ======");
  sort_robot_ordered_by_the_distance_with_starting_position();

  DEBUG("robot_affectations : " << robot_affectations);
  DEBUG("robot_consigns : " << robot_consigns);

  DEBUG("============= comp. ======");
  compute_robot_affectations_to_strategies();
  DEBUG("robot_affectations_by_strategy : " << robot_affectations_by_strategy);
  DEBUG("============= FIN======");
}

const std::string& Manager::get_next_strategy_with_goalie() const
{
  return strategy_with_goal;
}

void Manager::declare_and_assign_next_strategies(const std::list<std::string>& future_strats)
{
  declare_next_strategies(future_strats);  // This is needed to comput robot affectation
  DEBUG("Declare and assigne strat");
  for (const std::pair<std::string, std::vector<int> >& elem : robot_affectations_by_strategy)
  {
    const std::string& strategy_name = elem.first;
    const std::vector<int>& affectation = elem.second;
    bool have_a_goalie = (get_next_strategy_with_goalie() == strategy_name);
    assign_strategy(strategy_name, ai_data.time, affectation, have_a_goalie);
  }
}

RhobanSSLAnnotation::Annotations Manager::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;

  for (const std::string& strategy_name : current_strategy_names)
  {
    annotations.addAnnotations(get_strategy(strategy_name).get_annotations());
  };

  return annotations;
}

void Manager::set_ball_avoidance_for_all_robots(bool value = true)
{
  ai_data.force_ball_avoidance = value;
}

};  // namespace Manager
};  // namespace rhoban_ssl
