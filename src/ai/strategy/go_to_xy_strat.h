/*
    This file is part of SSL.

    Copyright 2018 RomainPC (romainpc.lechat@laposte.net)

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

#include <strategy/strategy.h>
#include <robot_behavior/tutorials/medium/follow_robot.h>
#include <robot_behavior/tutorials/beginner/goto_ball.h>
#include <robot_behavior/go_to_xy.h>

namespace rhoban_ssl
{
namespace strategy
{
class GoToXYStrat : public Strategy
{
private:
  bool behaviors_are_assigned_;

  int NB_ROBOT = 6;

  std::vector<std::shared_ptr<robot_behavior::GoToXY>> vect_go_to_xy_;
  std::vector<rhoban_geometry::Point> positions_;

public:
  GoToXYStrat();
  GoToXYStrat(std::vector<rhoban_geometry::Point> positions);
  virtual ~GoToXYStrat();

  virtual int minRobots() const;
  virtual int maxRobots() const;
  virtual GoalieNeed needsGoalie() const;

  static const std::string name;

  virtual void start(double time);
  virtual void stop(double time);

  virtual void update(double time);

  virtual void assignBehaviorToRobots(
      std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt);

  virtual std::list<std::pair<rhoban_geometry::Point, ContinuousAngle>>
  getStartingPositions(int number_of_avalaible_robots);
  virtual bool getStartingPositionForGoalie(rhoban_geometry::Point& linear_position, ContinuousAngle& angular_position);

  void setPositions(std::vector<rhoban_geometry::Point> positions);

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
};

};  // namespace strategy
};  // namespace rhoban_ssl