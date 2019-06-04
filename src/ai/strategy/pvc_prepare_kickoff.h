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
#include <robot_behavior/robot_behavior.h>
#include "placer.h"
#include "math/position.h"

namespace rhoban_ssl
{
namespace strategy
{
struct RobotPlacement
{
  bool goal_is_placed;
  std::vector<Position> field_robot_position;
  Position goalie_position;

  RobotPlacement();
  RobotPlacement(std::vector<Position> field_robot_position, Position goalie_position);
  RobotPlacement(std::vector<Position> field_robot_position);
};

class PrepareKickoff : public Strategy
{
private:
  bool is_kicking_;
  bool strategy_is_active_;
  RobotPlacement attacking_placement_;
  RobotPlacement defending_placement_;
  Placer placer_when_kicking_;
  Placer placer_when_no_kicking_;

  RobotPlacement defaultAttackingKickoffPlacement() const;
  RobotPlacement defaultDefendingKickoffPlacement() const;
  rhoban_geometry::Point relative2absolute(double x, double y) const;

public:
  PrepareKickoff();
  virtual ~PrepareKickoff();

  virtual int minRobots() const;
  virtual int maxRobots() const;
  virtual GoalieNeed needsGoalie() const;

  static const std::string name;

  virtual void start(double time);
  virtual void stop(double time);

  virtual void update(double time);
  void updateStartingPositions();

  virtual void assignBehaviorToRobots(
      std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt);

  void setPositions(const std::vector<int>& robot_affectations,
                    const std::vector<std::pair<rhoban_geometry::Point, ContinuousAngle> >& robot_consigns,
                    bool allly_have_the_kickoff);
  void setGoaliePositions(const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position,
                          bool allly_have_the_kickoff);

  void setKicking(bool value = true);

  virtual void setRobotAffectation(const std::vector<int>& robot_ids);

  virtual std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
  getStartingPositions(int number_of_avalaible_robots);
  virtual bool getStartingPositionForGoalie(rhoban_geometry::Point& linear_position, ContinuousAngle& angular_position);

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual void setGoalie(int id, bool to_be_managed);
};

};  // namespace strategy
};  // namespace rhoban_ssl
