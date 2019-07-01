
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

#include <strategy/strategy.h>

namespace rhoban_ssl
{
namespace strategy
{
class KeeperStrat : public Strategy
{
private:
  bool behaviors_are_assigned_;
  //  std::shared_ptr<robot_behavior::Degageur> degageur_;
  // std::shared_ptr<robot_behavior::Goalie> goalie_;

public:
  /**
   * @brief Constructor
   */
  KeeperStrat();
  /**
   * @brief Return minimal robot.
   * @return
   */
  virtual int minRobots() const;
  /**
   * @brief Return maximum robot.
   * @return
   */
  virtual int maxRobots() const;
  /**
   * @brief Destructor
   */
  virtual ~KeeperStrat();
  virtual GoalieNeed needsGoalie() const;

  static const std::string name;

  virtual void start(double time);
  virtual void stop(double time);

  virtual void update(double time);

  virtual void assignBehaviorToRobots(
      std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt);
  virtual std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
  getStartingPositions(int number_of_avalaible_robots) const;
  /**
   * @brief Set the starting point for the goalie.
   * @param linear_position Linear position to set for the goalie.
   * @param angular_position Angular position to set for the goalie.
   * @return has_position If you want to set a position when you start return true. Otherwise, return false.
   */
  virtual bool getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                            ContinuousAngle& angular_position) const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
};

};  // namespace strategy
};  // namespace rhoban_ssl
