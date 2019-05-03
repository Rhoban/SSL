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
 * @class AnnotationClosestRobotToTheBall
 * @brief Tutorial to show how to find the closest robot to the ball and draw the response on the viewer.
 *
 * Find the closest robot of the ball with the information of the class GameInformations
 * and compare the distance between the ally closestrobot and the opponent closest robot.
 */
class AnnotationClosestRobotToTheBall : public RobotBehavior
{
private:
  /**
   * @see rhoban_ssl::annotations::Annotations
   */
  rhoban_ssl::annotations::Annotations annotations_;

public:
  /**
   * @brief Constructor.
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @see ai::AiData
   */
  AnnotationClosestRobotToTheBall(ai::AiData& ai_data);

  /**
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   *
   * @see RobotBehavior::update
   */
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  /**
   * @see Control
   */
  virtual Control control() const;

  /**
   * @see RhobanSSLAnnotation::Annotations
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~AnnotationClosestRobotToTheBall();
};

}  // Namespace beginner
}  // Namespace robot_behavior
}  // namespace rhoban_ssl
