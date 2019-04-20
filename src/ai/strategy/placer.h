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

#include "strategy.h"
#include <string>
#include <list>
#include <robot_behavior/robot_behavior.h>

namespace rhoban_ssl
{
namespace strategy
{
class Placer : public Strategy
{
private:
  std::map<int, std::pair<rhoban_geometry::Point, ContinuousAngle> > player_positions_;

  std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> > starting_positions_;

public:
  virtual std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
  getStartingPositions(int number_of_avalaible_robots) const;
  void setStartingPosition(const std::vector<rhoban_geometry::Point>& starting_position);

private:
  rhoban_geometry::Point goalie_linear_position_;
  ContinuousAngle goalie_angular_position_;
  std::pair<rhoban_geometry::Point, ContinuousAngle> starting_position_for_goalie_;
  bool goalie_is_defined_;

public:
  virtual bool getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                            ContinuousAngle& angular_position) const;

  void setStartingPositionForGoalie(const rhoban_geometry::Point& linear_position,
                                    const ContinuousAngle& angular_position);

  Placer(ai::AiData& ai_data);
  bool behavior_has_been_assigned;
  int minRobots() const;
  int maxRobots() const;
  virtual GoalieNeed needsGoalie() const;

  // Try to place one robot at each given position.
  // This function return which robot id have been placed.
  // the order of robot id correspond to the order of the given robot position
  void setPositions(const std::vector<int>& robot_affectations,
                    const std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> >& robot_consigns);
  void setGoaliePositions(const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position);

  static const std::string name;

  void start(double time);
  void stop(double time);

  void assignBehaviorToRobots(std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior,
                              double time, double dt);
  virtual ~Placer();

  void setStartingPositions(const std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >& starting_positions_);
};

};  // namespace strategy
};  // namespace rhoban_ssl
