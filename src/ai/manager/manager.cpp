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
    along with SSL.  If not, see <http://www.gnu.org/licenses/.
*/

#include "manager.h"

#include <debug.h>
#include <strategy/halt.h>
#include <strategy/placer.h>
#include <algorithm>
#include <core/collection.h>
#include <core/print_collection.h>
#include <algorithm>
#include <math/matching.h>

namespace rhoban_ssl
{
namespace manager
{
void Manager::declareTeamIds(const std::vector<int>& team_ids)
{
  this->team_ids_ = team_ids;
}

const std::vector<int>& Manager::getTeamIds() const
{
  return team_ids_;
}

void Manager::registerStrategy(const std::string& strategy_name, std::shared_ptr<strategy::Strategy> strategy)
{
  assert(strategies_.find(strategy_name) == strategies_.end());
  strategies_[strategy_name] = strategy;
}

void Manager::clearStrategyAssignement()
{
  for (const std::string& name : current_strategy_names_)
  {
    getStrategy(name).stop(time());
  }
  current_strategy_names_.clear();
  assignStrategy(MANAGER__REMOVE_ROBOTS, time(), getInvalidTeamIds());
}

void Manager::assignStrategy(const std::string& strategy_name, double time, const std::vector<int>& robot_ids,
                             bool assign_goalie)
{
  assert(strategies_.find(strategy_name) != strategies_.end());  // The name of the strategy is not declared. Please
                                                                 // register them with register_strategy() (during the
                                                                 // initialisation of your manager for example).
  assert(not(assign_goalie) or (assign_goalie and
                                std::find(robot_ids.begin(), robot_ids.end(),
                                          Data::get()->referee.teams_info[Ally].goalkeeper_number) ==
                                    robot_ids.end()));  // If you declare that you are assigning a goal, you should not
                                                        // declar the goal id inside the list of field robots.

  current_strategy_names_.push_front(strategy_name);
  strategy::Strategy& strategy = getStrategy(strategy_name);

  if (not(static_cast<unsigned int>(strategy.minRobots()) <= robot_ids.size()))
  {
    DEBUG("We have not enough robot for the strategy : "
          << strategy_name << ". Number of robot requested : " << static_cast<unsigned int>(strategy.minRobots())
          << ", number or avalaible robot : " << robot_ids.size() << ".");
  }
  assert(static_cast<unsigned int>(strategy.minRobots()) <= robot_ids.size());

  strategy.setGoalie(Data::get()->referee.teams_info[Ally].goalkeeper_number, assign_goalie);
  strategy.setGoalieOpponent(Data::get()->referee.teams_info[Opponent].goalkeeper_number);
  strategy.setRobotAffectation(robot_ids);
  strategy.start(time);

  std::cout << "Manager: we assign '" << strategy_name << "' with " << robot_ids.size()
            << " field robots : " << strategy.getPlayerIds() << "."
            << " Goalie id is " << strategy.getGoalie() << " and is " << (assign_goalie ? "" : "not ")
            << "assigned to the strategy as goalie." << std::endl;
}

strategy::Strategy& Manager::getStrategy(const std::string& strategy_name)
{
  assert(strategies_.find(strategy_name) != strategies_.end());
  return *(strategies_.at(strategy_name));
}

const strategy::Strategy& Manager::getStrategy(const std::string& strategy_name) const
{
  assert(strategies_.find(strategy_name) != strategies_.end());
  return *(strategies_.at(strategy_name));
}

const std::list<std::string>& Manager::getCurrentStrategyNames() const
{
  return current_strategy_names_;
}

void Manager::updateStrategies(double time)
{
  for (std::pair<std::string, std::shared_ptr<strategy::Strategy> > elem : strategies_)
  {
    // REFACTO : TODO Remove the time passed.
    elem.second->update(time);
  }
}

void Manager::updateCurrentStrategies()
{
  for (const std::string& name : current_strategy_names_)
  {
    //@TODO : Remove the time passed.
    getStrategy(name).update(Data::get()->time.now());
  }
}

void Manager::assignBehaviorToRobots(std::map<int, std::shared_ptr<robot_behavior::RobotBehavior> >& robot_behaviors,
                                     double time, double dt)
{
  for (const std::string& name : current_strategy_names_)
  {
    getStrategy(name).assignBehaviorToRobots(
        [&](int id, std::shared_ptr<robot_behavior::RobotBehavior> behavior) {
#ifndef NDEBUG
          bool id_is_present = false;
          for (int robot_id : this->getStrategy(name).getPlayerIds())
          {
            if (robot_id == id)
            {
              id_is_present = true;
              break;
            }
          }
          if (this->getStrategy(name).haveToManageTheGoalie() and
              int(Data::get()->referee.teams_info[Ally].goalkeeper_number) == id)
          {
            id_is_present = true;
          }
// assert(id_is_present);
#endif
          if (id == -1)
            return;
          robot_behaviors[id] = behavior;
          return;
        },
        time, dt);
  }
}

Manager::Manager(std::string name) : manager_name_(name), blue_is_not_set_(true)
{
  registerStrategy(MANAGER__REMOVE_ROBOTS, std::shared_ptr<strategy::Strategy>(new strategy::Halt()));
  registerStrategy(MANAGER__PLACER, std::shared_ptr<strategy::Strategy>(new strategy::Placer()));
}

std::string Manager::name()
{
  return manager_name_;
}

double Manager::time() const
{
  return Data::get()->time.now();
}

double Manager::dt() const
{
  // return Data::get()->ai_data.dt;
  return ai::Config::period;
}

void Manager::affectInvalidRobotsToInvalidRobotsStrategy()
{
  strategy::Strategy& strategy = getStrategy(MANAGER__REMOVE_ROBOTS);
  strategy.stop(time());
  strategy.setRobotAffectation(getInvalidTeamIds());
  strategy.start(time());
}

void Manager::removeInvalidRobots()
{
  detectInvalidRobots();
  affectInvalidRobotsToInvalidRobotsStrategy();
}

void Manager::detectInvalidRobots()
{
  // TODO : we need to detect when the list of invalid robot change.
  // When it change, then, we need to reaffect robot ids.

  valid_team_ids_.clear();
  valid_player_ids_.clear();
  invalid_team_ids_.clear();

  for (int id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
  {
    if (Data::get()->robots[Ally][id].isActive())
    {
      valid_team_ids_.push_back(id);
      if (int(Data::get()->referee.teams_info[Ally].goalkeeper_number) != id)
      {
        valid_player_ids_.push_back(id);
      }
    }
    else
    {
      invalid_team_ids_.push_back(id);
    }
  }
}

const std::vector<int>& Manager::getValidTeamIds() const
{
  return valid_team_ids_;
}
const std::vector<int>& Manager::getValidPlayerIds() const
{
  return valid_player_ids_;
}
const std::vector<int>& Manager::getInvalidTeamIds() const
{
  return invalid_team_ids_;
}
Manager::~Manager()
{
}

std::vector<std::string> Manager::getAvailableStrategies()
{
  std::vector<std::string> strategyNames;

  for (auto& entry : strategies_)
  {
    strategyNames.push_back(entry.first);
  }

  return strategyNames;
}

std::list<std::string> Manager::determineTheRobotNeedsForTheStrategies(const std::list<std::string>& next_strategies)
{
  std::list<std::string> next_valid_strategies;
  minimal_nb_of_robots_to_be_affected_ = 0;
  strategy_with_arbitrary_number_of_robot_ = "";
  robot_affectations_by_strategy_.clear();
  goal_has_to_be_placed_ = false;
  for (const std::string& strategy_name : next_strategies)
  {
    // for players
    const strategy::Strategy& strategy = getStrategy(strategy_name);
    int minimal_number_of_robot = strategy.minRobots();
    if (minimal_number_of_robot + minimal_nb_of_robots_to_be_affected_ > getValidPlayerIds().size())
    {
      DEBUG("Strategy : " << strategy_name << " need too many robots. We remove them to the list of next strategies.");
      continue;
    }
    else
    {
      next_valid_strategies.push_back(strategy_name);
    }
    minimal_nb_of_robots_to_be_affected_ += minimal_number_of_robot;
    if (strategy.maxRobots() == -1)
    {
      // We want the last one, so we remove previous discovered strategy name
      strategy_with_arbitrary_number_of_robot_ = strategy_name;
    }
    robot_affectations_by_strategy_[strategy_name] = std::vector<int>(minimal_number_of_robot);

    // for goalie
    bool strategy_needs_a_goal =
        (strategy.needsGoalie() == strategy::GoalieNeed::YES) or
        ((getStrategy(strategy_name).needsGoalie() == strategy::GoalieNeed::IF_POSSIBLE) and (goal_has_to_be_placed_));
    if (strategy_needs_a_goal)
    {
      if (goal_has_to_be_placed_)
      {
        DEBUG("In strategy : " << strategy_name << ", a goalie is yet defined but the strategy : "
                               << strategy_with_goal_ << " is also assigned and have to contain a goal too.");
      }
      assert(not(goal_has_to_be_placed_));  // Two goal is defined, check you are not assigning two stratgies with a
                                            // goal
                                            // !
      goal_has_to_be_placed_ = true;
      strategy_with_goal_ = strategy_name;
    }
  }
  if (getValidPlayerIds().size() < minimal_nb_of_robots_to_be_affected_)
  {
    nb_of_extra_robots_ = 0;
  }
  else
  {
    nb_of_extra_robots_ = (getValidPlayerIds().size() - minimal_nb_of_robots_to_be_affected_);
  }
  if (strategy_with_arbitrary_number_of_robot_ != "")
  {
    robot_affectations_by_strategy_[strategy_with_arbitrary_number_of_robot_].resize(
        robot_affectations_by_strategy_[strategy_with_arbitrary_number_of_robot_].size() + nb_of_extra_robots_);
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

void Manager::aggregateAllStartingPositionOfAllStrategies(const std::list<std::string>& next_strategies)
{
  starting_positions_.clear();
  repartitions_of_starting_positions_in_the_list_.clear();

  // For the players
  for (const std::string& strategy_name : next_strategies)
  {
    const strategy::Strategy& strategy = getStrategy(strategy_name);
    std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> > startings =
        strategy.getStartingPositions(robot_affectations_by_strategy_.at(strategy_name).size());
    starting_positions_.insert(starting_positions_.end(), startings.begin(), startings.end());
    repartitions_of_starting_positions_in_the_list_.push_back(
        std::pair<std::string, int>(strategy_name, startings.size()));
  }

  // For the goalie
  if (goal_has_to_be_placed_)
  {
    if (!getStrategy(strategy_with_goal_)
             .getStartingPositionForGoalie(this->goalie_linear_position_, this->goalie_angular_position_))
    {
      this->goalie_linear_position_ = rhoban_geometry::Point(-Data::get()->field.field_length / 2.0, 0.0);
      this->goalie_angular_position_ = ContinuousAngle(0.0);
    }
  }
}

void Manager::sortRobotOrderedByTheDistanceWithStartingPosition()
{
  assert(starting_positions_.size() <= getValidPlayerIds().size());
  robot_consigns_ = std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> >(getValidPlayerIds().size());
  robot_affectations_.resize(getValidPlayerIds().size());

  std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> > choising_positions =
      list2vector(starting_positions_);

  std::function<double(const int& robot_id, const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos)>
      robot_ranking = [this](const int& robot_id, const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos) {
        return Vector2d(pos.first - Data::get()->robots[Ally][robot_id].getMovement().linearPosition(time()))
            .normSquare();
      };

  std::function<double(const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos, const int& robot_id)>
      distance_ranking = [this](const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos, const int& robot_id) {
        return Vector2d(pos.first - Data::get()->robots[Ally][robot_id].getMovement().linearPosition(time()))
            .normSquare();
      };

  matching::Matchings matchings = matching::galeShapleyAlgorithm(getValidPlayerIds(), choising_positions, robot_ranking,
                                                                 distance_ranking, false, false);

  std::list<int> not_choosen_robot;
  for (unsigned int id : matchings.unaffected_man)
  {
    not_choosen_robot.push_back(getValidPlayerIds()[id]);
  }

  for (unsigned int i = 0; i < choising_positions.size(); i++)
  {
    const std::pair<rhoban_geometry::Point, ContinuousAngle>& pos = choising_positions[i];
    robot_consigns_[i] = pos;
    robot_affectations_[i] = getValidPlayerIds()[matchings.women_to_man_matchings.at(i)];
  }

  // We place the other robot outsde the field.
  std::list<int>::const_iterator it = not_choosen_robot.begin();
  for (unsigned int i = starting_positions_.size(); i < getValidPlayerIds().size(); i++)
  {
    robot_consigns_[i] = std::pair<rhoban_geometry::Point, ContinuousAngle>(
        rhoban_geometry::Point(
            -((5.0 * ai::Config::robot_radius) * (i - starting_positions_.size()) + 1.5 * ai::Config::robot_radius),
            -Data::get()->field.field_width / 2.0 + ai::Config::robot_radius),
        ContinuousAngle(0.0));
    robot_affectations_[i] = *it;
    it++;
  }
}

void Manager::computeRobotAffectationsToStrategies()
{
  unsigned int cpt_robot = 0;
  unsigned int cpt_extra_robot = 0;
  for (const std::pair<std::string, int>& elem : repartitions_of_starting_positions_in_the_list_)
  {
    const std::string& strategy_name = elem.first;
    const int nb_robots = elem.second;
    for (int i = 0; i < nb_robots; i++)
    {
      robot_affectations_by_strategy_[strategy_name][i] = robot_affectations_.at(cpt_robot + i);
    }

    int nb_robots_with_no_startings = (robot_affectations_by_strategy_.at(strategy_name).size() - nb_robots);

    for (int i = 0; i < nb_robots_with_no_startings; i++)
    {
      robot_affectations_by_strategy_[strategy_name][nb_robots + i] =
          robot_affectations_.at(starting_positions_.size() + cpt_extra_robot + i);
    }
    cpt_robot += nb_robots;
    cpt_extra_robot += nb_robots_with_no_startings;
  }
}

void Manager::declareRobotPositionsInThePlacer()
{
  if (goal_has_to_be_placed_)
  {
    getStrategy<strategy::Placer>(MANAGER__PLACER)
        .setGoaliePositions(goalie_linear_position_, goalie_angular_position_);
  }
  // else{
  // TODO : should we declare in the strategy that goalie have to be ignored ?
  //}

  assert(starting_positions_.size() <= getValidTeamIds().size());

  getStrategy<strategy::Placer>(MANAGER__PLACER).setPositions(robot_affectations_, robot_consigns_);
}

void Manager::placeAllTheRobots(double time, const std::list<std::string>& next_strategies)
{
  declareNextStrategies(next_strategies);
  declareRobotPositionsInThePlacer();
  assignStrategy(MANAGER__PLACER, time, getValidPlayerIds(), goal_has_to_be_placed_);
}

const std::vector<int>& Manager::getRobotAffectations(const std::string& strategy_name) const
{
  assert(robot_affectations_by_strategy_.find(strategy_name) != robot_affectations_by_strategy_.end());
  return robot_affectations_by_strategy_.at(strategy_name);
}

void Manager::declareNextStrategies(const std::list<std::string>& next_strategies)
{
  DEBUG("");
  DEBUG("#########################");
  DEBUG("#########################");
  DEBUG("DECLARE NEXT STRATEGIES");
  DEBUG("#########################");
  DEBUG("#########################");
  DEBUG("next_strategies : " << next_strategies);

  std::list<std::string> next_valid_strategies = determineTheRobotNeedsForTheStrategies(next_strategies);

  DEBUG("============== determ. ======");
  // DEBUG("nb_of_extra_robots_non_affected : " << nb_of_extra_robots_non_affected_);
  DEBUG("minimal_nb_of_robots_to_be_affected : " << minimal_nb_of_robots_to_be_affected_);
  DEBUG("nb_of_extra_robots : " << nb_of_extra_robots_);
  DEBUG("strategy_with_arbitrary_number_of_robot : " << strategy_with_arbitrary_number_of_robot_);
  DEBUG("goal_has_to_be_placed : " << goal_has_to_be_placed_);
  DEBUG("strategy_with_goal : " << strategy_with_goal_);
  DEBUG("robot_affectations_by_strategy : " << robot_affectations_by_strategy_);

  DEBUG("============== aggre. ======");
  aggregateAllStartingPositionOfAllStrategies(next_valid_strategies);

  DEBUG("starting_positions : " << starting_positions_);
  DEBUG("repartitions_of_starting_positions_in_the_list : " << repartitions_of_starting_positions_in_the_list_);

  DEBUG("goalie_linear_position : " << goalie_linear_position_);
  DEBUG("goalie_angular_position : " << goalie_angular_position_);

  DEBUG("============== sort. ======");
  sortRobotOrderedByTheDistanceWithStartingPosition();

  DEBUG("robot_affectations : " << robot_affectations_);
  DEBUG("robot_consigns : " << robot_consigns_);

  DEBUG("============= comp. ======");
  computeRobotAffectationsToStrategies();
  DEBUG("robot_affectations_by_strategy : " << robot_affectations_by_strategy_);
  DEBUG("============= FIN======");
}

const std::string& Manager::getNextStrategyWithGoalie() const
{
  return strategy_with_goal_;
}

void Manager::declareAndAssignNextStrategies(const std::list<std::string>& future_strats)
{
  declareNextStrategies(future_strats);  // This is needed to comput robot affectation
  DEBUG("Declare and assigne strat");
  for (const std::pair<std::string, std::vector<int> >& elem : robot_affectations_by_strategy_)
  {
    const std::string& strategy_name = elem.first;
    const std::vector<int>& affectation = elem.second;
    bool have_a_goalie = (getNextStrategyWithGoalie() == strategy_name);
    assignStrategy(strategy_name, Data::get()->time.now(), affectation, have_a_goalie);
  }
}

rhoban_ssl::annotations::Annotations Manager::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;

  for (const std::string& strategy_name : current_strategy_names_)
  {
    annotations.addAnnotations(getStrategy(strategy_name).getAnnotations());
  };

  return annotations;
}

void Manager::setBallAvoidanceForAllRobots(bool value = true)
{
  Data::get()->ai_data.force_ball_avoidance = value;
}

Json::Value Manager::getProperties()
{
  // @ TODO : Create a default behavior
  return Json::Value();
}

void Manager::setProperties(Json::Value)
{
}

}  // namespace manager
}  // namespace rhoban_ssl
