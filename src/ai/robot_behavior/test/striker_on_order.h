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

#include <robot_behavior/robot_behavior.h>
#include <robot_behavior/factory.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace test
{
/**
 * @brief The StrikerOnOrder class
 *
 * Go at a distance of run_up from the ball and shot when given the order.
 */
class StrikerOnOrder : public RobotBehavior
{
private:
  /**
   * @brief The power of the kicker.
   */
  const double power_kicker_;
  /**
   * @brief Distance needed to shoot.
   */
  const double run_up_;
  /**
   * @brief Target point to shoot.
   */
  const rhoban_geometry::Point target_;

  /**
   * @see rhoban_ssl::Robot_behavior::ConsignFollower
   */
  ConsignFollower* follower_;
  /**
   * Not use in this package but set in a case of copy.
   * @see rhoban_ssl::annotations::Annotations
   */
  rhoban_ssl::annotations::Annotations annotations_;
  /**
   * @brief Is the striker is allowed to strike ?
   */
  bool allow_to_strike_;
  /**
   * @brief Is the robot is well placed ?
   */
  bool is_placed_;
  /**
   * @brief Has the robot strike ?
   */
  bool has_strike_;
  /**
   * @brief The minimal distance when the behavior say the robot is placed.
   */
  double dist_robot_is_placed_;
public:
  /**
   * @brief Constructor
   * @param ai_data
   * @see ai::AiData
   */
  StrikerOnOrder(ai::AiData& ai_data, const double power_kicker, const double run_up,
                 const rhoban_geometry::Point target, const bool allow_to_strike);
  /**
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   */
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);
  /**
   * @see Control.
   */
  virtual Control control() const;

  /**
   * @see rhoban_ssl::annotations::Annotations
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   * @brief Destructor.
   */
  virtual ~StrikerOnOrder();

  /**
   * @brief Change the allow_to_strike.
   * @param allow_to_strike The new value.
   */
  void setAllowToStrike(const bool allow_to_strike);
};
}  // namespace test
}  // namespace robot_behavior
}  // namespace rhoban_ssl
