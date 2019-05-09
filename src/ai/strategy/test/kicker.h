/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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
#include <robot_behavior/test/striker_on_order.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace strategy
{
namespace test
{
class Kicker : public Strategy
{
private:
  rhoban_ssl::annotations::Annotations annotations_;

  bool behaviors_are_assigned_;

  /**
   * @brief Target point to shoot.
   */
  const rhoban_geometry::Point target_;

  /**
   * @brief power of the shoot.
   */
  const double power_;

  /**
   * @brief The run_up_ when you prepare to strike.
   */
  const double run_up_;

  /**
   * @brief Use in error_on_line.
   * If not set, the default is the perpendicular of the direction.
   */
  Vector2d line_imaginary_;

public:
  /**
   * @brief Name of the strategy.
   */
  static const std::string name;
  /**
   * @brief Constructor.
   * @param Ai data
   * @see ai::AiData
   */
  Kicker(ai::AiData& ai_data, rhoban_geometry::Point target, double power, double run_up, Vector2d line_imaginary = Vector2d(0.0,0.0));

  /**
   * @brief The minimum number of robots needs for the strategy.
   * @return min The minimum number.
   */
  virtual int minRobots() const;
  /**
   * @brief The maximum number of robots needs for the strategy.
   * @return max The maximum number.
   */
  virtual int maxRobots() const;

  /**
   * @brief Set if the goalie needs.
   * @return
   */
  virtual GoalieNeed needsGoalie() const;
  /**
   * @brief Function call when the strategy is started.
   * @param time
   */
  virtual void start(double time);
  /**
   * @brief Function call when the strategy is stopped.
   * @param time
   */
  virtual void stop(double time);

  /**
   * @brief Function call when the strategy is updated.
   * @param time
   */
  virtual void update(double time);

  /**
   * @brief Assign behavior to robots
   * @todo Finish the params.
   * @param assign_behavior
   * @param time
   * @param dt
   */
  virtual void assignBehaviorToRobots(
      std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt);

  /**
   * @todo : Finish the commentary.
   * @brief Get the starting points of all robots.
   * @param number_of_avalaible_robots The number of avalaible of robots.
   * @return
   */
  virtual std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
  getStartingPositions(int number_of_avalaible_robots);

  /**
   * @brief Get the starting points of goalie robot.
   * @param linear_position
   * @param angular_position
   * @return
   */
  virtual bool getStartingPositionForGoalie(rhoban_geometry::Point& linear_position, ContinuousAngle& angular_position);

  /**
   * @brief Get Annotations.
   * @return
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   * @brief Destructor
   */
  virtual ~Kicker();
};
}  // namespace test
}  // namespace strategy
}  // namespace rhoban_ssl
