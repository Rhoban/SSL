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
#include <robot_behavior/robot_follower.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
/**
 * @class Striker
 * @brief Tutorial class to show how to strike the ball.
 */
class Striker : public RobotBehavior
{
private:
  /**
   * @brief The target corner which the robot goes.
   */
  ConsignFollower* follower_;
  /**
   * Not use in this package but set in a case of copy.
   * @see rhoban_ssl::annotations::Annotations
   */
  rhoban_ssl::annotations::Annotations annotations_;
  /**
   * @brief The target point which the robot strike the ball.
   */
  rhoban_geometry::Point striking_point_;
  /**
   * @brief Boolean to see if the robot is well placed or not.
   */
  bool placed_;

public:
  /**
   * @brief Constructor.
   * The default value of the target_corner_ is set to left opponent corner.
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @see ai::AiData
   */
  Striker(ai::AiData& ai_data);

  /**
   * @brief Place with the algorithm of Prepare striker and strike the ball.
   *
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   * @see rhoban_ssl::robot_behavior::PrepareStriker
   */
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);
  /**
   *  @see Control.
   */
  virtual Control control() const;
  /**
   * @see rhoban_ssl::annotations::Annotations
   * The class don't draw any annotations.
   * The follower draw annotation.
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
  /**
   * @brief Destructor.
   */
  virtual ~Striker();
};

};  // namespace medium
};  // namespace   robot_behavior
};  // namespace rhoban_ssl
