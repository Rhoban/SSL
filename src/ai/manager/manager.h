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

#pragma once

#include <game_informations.h>
#include <strategy/strategy.h>
#include <robot_behavior/robot_behavior.h>
#include <map>
#include <memory>
#include <vector>
#include <annotations/annotations.h>

namespace rhoban_ssl
{
namespace manager
{
/**
 *@brief In our IA model, Managers control Strategies wich control basic Behaviors.
 */
class Manager
{
public:
  static constexpr const char* MANAGER__REMOVE_ROBOTS = "manager__remove_robots";
  static constexpr const char* MANAGER__PLACER = "manager__placer";

private:
  std::string manager_name_;

  bool blue_is_not_set_;
  bool blue_team_on_positive_half_;

  /**
   * @brief ids of all ally robots
   */
  std::vector<int> team_ids_;

  /**
   * @brief ids of valid ally robots
   */
  std::vector<int> valid_team_ids_;

  /**
   * @brief ids of valid ally robots, except goalie
   */
  std::vector<int> valid_player_ids_;

  /**
   * @brief ids of invalid ally robots
   */
  std::vector<int> invalid_team_ids_;

  /**
   * @brief string list, the names of currently applied strategies.
   */
  std::list<std::string> current_strategy_names_;
  /**
   * @brief map which associate a registered strategy name to the corresponding Strategy class pointer.
   */
  std::map<std::string, std::shared_ptr<strategy::Strategy>> strategies_;

  /**
   * @brief affect invalid robot to the special strategy MANAGER__REMOVE_ROBOTS.
   */
  void affectInvalidRobotsToInvalidRobotsStrategy();

  /**
   * @brief refresh lists of ids by asking GlobalData
   */
  void detectInvalidRobots();

public:
  /**
   * @brief Constructor.
   * @param name
   */
  Manager(std::string name);

  /**
   * @brief returns the name of the manager
   *
   * The name of all manager are define in the factory and set
   * at the construction.
   * @return a string
   */
  std::string name();

  /**
   * @brief getAvailableStrategies
   * @return all name of available strategies in a vector
   */
  std::vector<std::string> getAvailableStrategies();

  double time() const;
  int dt() const;

  /**
   * @brief get the name of your team
   *
   * @return name
   */
  const std::string& getTeamName() const;
  /**
   * @brief set team ids vector of the manager
   *
   * @param team_ids : a vector of integers.
   */
  void declareTeamIds(const std::vector<int>& team_ids);
  const std::string& getNextStrategyWithGoalie() const;

  /**
   * @brief get team ids vector of the manager
   *
   * @return a vector of integers.
   */
  const std::vector<int>& getTeamIds() const;

  /**
   * @brief get valid team ids vector of the manager
   *
   * @return a vector of integers.
   */
  const std::vector<int>& getValidTeamIds() const;

  /**
   * @brief get valid player ids vector of the manager
   *
   * @return a vector of integers.
   */
  const std::vector<int>& getValidPlayerIds() const;

  /**
   * @brief get invalid team ids vector of the manager
   *
   * @return a vector of integers.
   */
  const std::vector<int>& getInvalidTeamIds() const;

  template <typename STRATEGY>
  STRATEGY& getStrategy(const std::string& name)
  {
    return static_cast<STRATEGY&>(*strategies_.at(name));
  };

  template <typename STRATEGY>
  STRATEGY& getStrategy()
  {
    return getStrategy<STRATEGY>(STRATEGY::name);
  };

  /**
   * @brief get a strategy object by reading the map of shared pointers
   * @param strategy_name
   * @return the strategy object
   */
  strategy::Strategy& getStrategy(const std::string& strategy_name);
  const strategy::Strategy& getStrategy(const std::string& strategy_name) const;

  /**
   * @brief get names of currently applied strategies
   * @return list of names
   */
  const std::list<std::string>& getCurrentStrategyNames() const;

  /**
   * @brief to add a new element in the strategy register
   *
   * @param strategy_name : the key, the strategy name
   * @param strategy : the corresponding pointer
   */
  void registerStrategy(const std::string& strategy_name, std::shared_ptr<strategy::Strategy> strategy);

  /**
   * @brief remove all currently assigned strategies.
   */
  void clearStrategyAssignement();

  /**
   * @brief apply a registered strategy to a certain number of robots.
   *
   * @param strategy_name
   * @param time : the time
   * @param robot_ids : a vector of robots to apply the strategy
   * @param assign_goalie : boolean to activate to define a goalie strategy
   */
  void assignStrategy(const std::string& strategy_name, double time, const std::vector<int>& robot_ids,
                      bool assign_goalie = false);

  /**
   * @brief compute needed robots and assign a list of strategies.
   *
   * @param future_strats
   */
  void declareAndAssignNextStrategies(const std::list<std::string>& future_strats);

  virtual void update() = 0;

  /**
   * @brief update() each registered strategies.
   */
  virtual void updateStrategies(double time);

  /**
   * @brief update() each of currently applied strategies.
   */
  virtual void updateCurrentStrategies();

  virtual void assignBehaviorToRobots(std::map<int, std::shared_ptr<robot_behavior::RobotBehavior>>& robot_behaviors,
                                      double time, double dt);

  /**
   * @brief detect invalid robots and affect them to the invalid strategy
   */
  void removeInvalidRobots();

  virtual ~Manager();

private:
  /**
   * @brief by considering how many robots are valid, set differents variable.
   *
   * @param next_strategies : a list of strategies
   * @return a list of valid strategies among those entered
   */
  std::list<std::string> determineTheRobotNeedsForTheStrategies(const std::list<std::string>& next_strategies);
  // unsigned int nb_of_extra_robots_non_affected_;
  /**
   * @brief number minimal of robots absolutely necessary, used by determineTheRobotNeedsForTheStrategies()
   */
  unsigned int minimal_nb_of_robots_to_be_affected_;
  /**
   * @brief number of additionals robots not necessarily necessary, used by determineTheRobotNeedsForTheStrategies()
   */
  unsigned int nb_of_extra_robots_;
  std::string strategy_with_arbitrary_number_of_robot_;
  // init of robot_affectations_by_strategy;
  bool goal_has_to_be_placed_;
  std::string strategy_with_goal_;

  void aggregateAllStartingPositionOfAllStrategies(const std::list<std::string>& next_strategies);
  std::list<std::pair<rhoban_geometry::Point, ContinuousAngle>> starting_positions_;
  std::list<std::pair<std::string, int>> repartitions_of_starting_positions_in_the_list_;
  rhoban_geometry::Point goalie_linear_position_;
  ContinuousAngle goalie_angular_position_;

  void sortRobotOrderedByTheDistanceWithStartingPosition();
  std::vector<int> robot_affectations_;
  // this list is a special orrder of get_valid_player_ids().
  // ths starting_posiitons.size() fisrt robots of robot_affectation
  // correspond a good affectation with respect to the starting position
  // of startings position;
  std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle>> robot_consigns_;
  // this list contains the starting position of get_valid_player_ids().
  // the starting_posiitons.size() first position are equals to the position
  // pf starting_position.
  // the other are some default placement.

  void computeRobotAffectationsToStrategies();
  std::map<std::string, std::vector<int>> robot_affectations_by_strategy_;

  /*

      void determine_the_robot_needs_for_the_strategies();
          std::map<std::string, std::vector<int>> robot_affectations_by_strategy;
          std::map<std::string, int> number_of_extra_robot_by_strategy;
          unsigned int nb_of_extra_robots;
          unsigned int nb_of_extra_robots_non_affected;
      void compute_robot_affectations_to_strategies();
          unsigned int extra_robots = 0;

  */

  void declareRobotPositionsInThePlacer();

protected:
  /**
   * @brief by giving to this function a list of wanted strategies, it will count how many robots are used, decide what
   * is possible and calcul the best robot affectation because APB c'est bien.
   *
   * @param next_strategies
   */
  void declareNextStrategies(const std::list<std::string>& next_strategies);

public:
  void setBallAvoidanceForAllRobots(bool value);

  /**
   * @brief place all robots by assigning all roles and position for all strategies.
   *
   * @param time : the time
   * @param next_strategies : a list of strategies which will be used
   */
  void placeAllTheRobots(double time, const std::list<std::string>& next_strategies);
  const std::vector<int>& getRobotAffectations(const std::string& strategy_name) const;

  /**
   *
   * @brief : This function is used to draw annotations in the viewer.
   * You can use it to print what you want.
   *
   * For example :
   *
   *
   *  RhobanSSLAnnotation::Annotations get_annotations() const{
   *      RhobanSSLAnnotation::Annotations annotations;
   *      static double d = 0;
   *      d += 0.01;
   *
   *      annotations.addCircle(3, 3, 1, "cyan");
   *      annotations.addArrow(0, 0, cos(d), sin(d)*2, "magenta", true);
   *      return annotations;
   *  }
   *
   * @return see @rhoban_ssl::annotations::Annotations
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual Json::Value getProperties();

  virtual void setProperties(Json::Value);
};

}  // namespace manager
}  // namespace rhoban_ssl
