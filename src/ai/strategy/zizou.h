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
#include <robot_behavior/go_to_xy.h>
#include <robot_behavior/tutorials/beginner/goto_ball.h>
#include <robot_behavior/poke_ball.h>

namespace rhoban_ssl
{
namespace strategy
{
class Zizou : public Strategy
{
private:
  const double DISTANCE_MAX = 0.4;  // meters
  const double TIME_MAX = 2;        // seconds
  bool behaviors_are_assigned_;
  int state_ = 0;
  double offset_ = 0.8;
  double goto_coef_ = 0.4;

  std::shared_ptr<robot_behavior::beginner::GotoBall> goto_ball_;
  std::shared_ptr<robot_behavior::GoToXY> goto_xy_;
  std::shared_ptr<robot_behavior::PokeBall> poke_ball_;

  rhoban_geometry::Point goto_xy_direction_;
  rhoban_geometry::Point poke_direction_;

  int id_;

  rhoban_geometry::Point position_initial_;
  bool first_state_1_ = true;
  double time_initial_;

public:
  Zizou();
  virtual ~Zizou();

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

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
};

};  // namespace strategy
};  // namespace rhoban_ssl
