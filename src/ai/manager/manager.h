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
class Manager : public GameInformations
{
public:
  static constexpr const char* MANAGER__REMOVE_ROBOTS = "manager__remove_robots";
  static constexpr const char* MANAGER__PLACER = "manager__placer";

private:
  bool blue_is_not_set_;
  bool blue_team_on_positive_half_;

  std::vector<int> team_ids_;
  std::vector<int> valid_team_ids_;
  std::vector<int> valid_player_ids_;
  std::vector<int> invalid_team_ids_;

  std::list<std::string> current_strategy_names_;
  std::map<std::string, std::shared_ptr<strategy::Strategy>> strategies_;

  void affectInvalidRobotsToInvalidRobotsStrategy();
  void detectInvalidRobots();

public:
  double time() const;
  int dt() const;

  std::vector<std::string> getAvailableStrategies();

  Manager();

  const std::string& getTeamName() const;
  void declareTeamIds(const std::vector<int>& team_ids);
  const std::string& getNextStrategyWithGoalie() const;
  const std::vector<int>& getTeamIds() const;
  const std::vector<int>& getValidTeamIds() const;
  const std::vector<int>& getValidPlayerIds() const;
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

  strategy::Strategy& getStrategy(const std::string& strategy_name);
  const strategy::Strategy& getStrategy(const std::string& strategy_name) const;
  const std::list<std::string>& getCurrentStrategyNames() const;

  void registerStrategy(const std::string& strategy_name, std::shared_ptr<strategy::Strategy> strategy);

  void clearStrategyAssignement();

  void assignStrategy(const std::string& strategy_name, double time, const std::vector<int>& robot_ids,
                      bool assign_goalie = false);
  void declareAndAssignNextStrategies(const std::list<std::string>& future_strats);

  virtual void update() = 0;

  virtual void updateStrategies(double time);
  virtual void updateCurrentStrategies();

  virtual void assignBehaviorToRobots(std::map<int, std::shared_ptr<robot_behavior::RobotBehavior>>& robot_behaviors,
                                      double time, double dt);

  void changeAllyAndOpponentGoalieId();

  void changeTeamAndPointOfView(bool blue_have_it_s_goal_on_positive_x_axis);

  void removeInvalidRobots();

  virtual ~Manager();

private:
  std::list<std::string> determineTheRobotNeedsForTheStrategies(const std::list<std::string>& next_strategies);
  // unsigned int nb_of_extra_robots_non_affected_;
  unsigned int minimal_nb_of_robots_to_be_affected_;
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
  void declareNextStrategies(const std::list<std::string>& next_strategies);

public:
  void setBallAvoidanceForAllRobots(bool value);

  void placeAllTheRobots(double time, const std::list<std::string>& next_strategies);
  const std::vector<int>& getRobotAffectations(const std::string& strategy_name) const;

  //
  // This function is used to draw annotations in the viewer.
  // You can use it to print what you want.
  //
  // For example :
  //
  //
  //  RhobanSSLAnnotation::Annotations get_annotations() const{
  //      RhobanSSLAnnotation::Annotations annotations;
  //      static double d = 0;
  //      d += 0.01;
  //
  //      annotations.addCircle(3, 3, 1, "cyan");
  //      annotations.addArrow(0, 0, cos(d), sin(d)*2, "magenta", true);
  //      return annotations;
  //  }
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual Json::Value getProperties();

  virtual void setProperties(Json::Value);
};

}  // namespace manager
}  // namespace rhoban_ssl
