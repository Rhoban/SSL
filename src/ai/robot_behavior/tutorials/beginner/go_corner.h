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
namespace beginner
{
/**
 * @class GoCorner
 * @brief Tutorial to show how to move a robot in the side corner.
 */
class GoCorner : public RobotBehavior
{
private:
  /**
   * @see rhoban_ssl::Robot_behavior::ConsignFollower
   */
  ConsignFollower* follower_;
  /**
   * Not use in this package but set in a case of copy.
   * @see RhobanSSLAnnotation::Annotations
   */
  rhoban_ssl::annotations::Annotations annotations_;
  /**
   * @brief The target corner which the robot goes.
   */
  const rhoban_geometry::Point target_corner_;

public:
  /**
   * @brief Constructor.
   * The default value of the target_corner_ is set to left opponent corner.
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @see ai::AiData
   */
  GoCorner(ai::AiData& ai_data_);

  /**
   * @brief Set the position of the robot in the target_corner_.
   *
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   */
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  /**
   * Return the control of the behavior.
   */
  virtual Control control() const;

  /**
   * @see RhobanSSLAnnotation::Annotations
   * The class don't draw any annotations.
   * The follower draw annotation.
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   * @brief Destructor.
   */
  virtual ~GoCorner();
};
}  // namespace beginner
}  // namespace Robot_behavior
}  // namespace rhoban_ssl
