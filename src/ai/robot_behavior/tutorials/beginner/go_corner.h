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

#ifndef __ROBOT_BEHAVIOR__TUTORIALS__BEGINNER__GO__CORNER__
#define __ROBOT_BEHAVIOR__TUTORIALS__BEGINNER__GO__CORNER__

/** @file go_corner.h
 *  @brief Tutorial to show how to move a robot in the side corner.
 *  @author Etienne Schmitz (hello.etienne-schmitz.com)
 *  @version 1.0
 *  @date 17/03/2019
 */

#include <robot_behavior/robot_behavior.h>
#include <robot_behavior/factory.h>
/** @namespace RhobanSSL
 *
 * Namespace gather all class of the project.
 */
namespace RhobanSSL
{
/** @namespace Robot_behavior
 *
 * Namespace gather all robot behavior.
 */
namespace Robot_behavior
{
/** @namespace Robot_behavior
 *
 * Namespace gather all beginner tutorial.
 */
namespace beginner
{
/** @class GoCorner
 *  @brief Class Represent the tutorial behavior to go in corner.
 */
class GoCorner : public RobotBehavior
{
private:
  /** @brief The follower of the class.
   */
  ConsignFollower* follower_;
  /** @brief The annotation shows in the viewer.
   * Not use in this package.
   */
  RhobanSSLAnnotation::Annotations annotations_;
  /** @brief The target corner which the robot goes.
   *
   * The default value is the left opponent corner.
   */
  const rhoban_geometry::Point& target_corner_;

public:
  /** @brief Constructor of the class.
   *
   * @param ai_data : The AI data.
   */
  GoCorner(Ai::AiData& ai_data);

  /** @brief Set the position of the robot in the target_corner_.
   *
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot.
   * @param ball : The information for the ball.
   */
  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  /** Return the control of the behavior.
   */
  virtual Control control() const;

  /** Get all annotations for the viewer.
   *
   * The follower draw annotations.
   * The class don't draw any annotations.
   */
  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  /** @brief Destructor of the class.
   */
  virtual ~GoCorner();
};

};  // namespace beginner
};  // namespace Robot_behavior
};  // namespace RhobanSSL

#endif
