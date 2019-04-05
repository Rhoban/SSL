/*
    This file is part of SSL.

    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)

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

#include "robot_behavior/robot_behavior.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace beginner
{
/**
 * @class AnnotationsRobotCoords
 * @brief Tutorial to show how to print on the viewer near the robot its coordinates.
 */
class AnnotationsRobotCoords : public RobotBehavior
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
  AnnotationsRobotCoords(ai::AiData& ai_data);

  /**
   * @brief Update
   *
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
   * @see rhoban_ssl::annotations::Annotations
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   *  @brief Destructor.
   */
  virtual ~AnnotationsRobotCoords();
};

};  // namespace beginner
};  // namespace robot_behavior
};  // namespace rhoban_ssl
