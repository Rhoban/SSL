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
#include <memory>

namespace rhoban_ssl
{
namespace strategy
{
class FromRobotBehavior : public Strategy
{
private:
  std::function<std::shared_ptr<robot_behavior::RobotBehavior>(double time, double dt)> robot_behavior_allocator_;
  bool behavior_has_been_assigned_;
  bool is_goalie_;

  struct StartingPosition
  {
    bool is_defined;
    rhoban_geometry::Point linear_position;
    ContinuousAngle angular_position;
    StartingPosition() : is_defined(false){};
  };
  StartingPosition starting_position_;

public:
  FromRobotBehavior(
      std::function<std::shared_ptr<robot_behavior::RobotBehavior>(double time, double dt)> robot_behavior_allocator_,
      bool is_goalie_ = false);
  FromRobotBehavior(
      std::function<std::shared_ptr<robot_behavior::RobotBehavior>(double time, double dt)> robot_behavior_allocator,
      const rhoban_geometry::Point& starting_linear_position, const ContinuousAngle& starting_angular_position,
      bool is_goalie = false);

  virtual int minRobots() const;
  virtual int maxRobots() const;

  void setStartingPosition(const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position);

  static const std::string name;

  virtual void start(double time);
  virtual void stop(double time);

  virtual GoalieNeed needsGoalie() const;

  virtual void assignBehaviorToRobots(
      std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt);

  virtual std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
  getStartingPositions(int number_of_avalaible_robots) const;
  virtual bool getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                            ContinuousAngle& angular_position) const;

  virtual ~FromRobotBehavior();
};

};  // namespace strategy
};  // namespace rhoban_ssl
