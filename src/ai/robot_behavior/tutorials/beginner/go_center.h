/*
    This file is part of SSL.

    // REVIEW ES : Même chose que le .cpp file
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
 * @class GoCenter
 * @class Tutorial to show how to move a robot to the field center.
 */
class GoCenter : public RobotBehavior
{
private:
  /**
   * @see rhoban_ssl::Robot_behavior::ConsignFollower
   */
  ConsignFollower* follower_;
  /**
   * Not used in this package but set in case of copying.
   * @see rhoban_ssl::annotations::Annotations
   */
  rhoban_ssl::annotations::Annotations annotations_;
  
public:
  /**
   * @brief Constructor.
   * @param ai_data : the Robot Behavior needs the data from the AI.
   * @see ai:AiData
   */
  GoCenter(ai::AiData& ai_data);

  /**
   * @brief set the position of the robot to the center of the field.
   *
   * We use parameters to update the time and the position before doing anything.
   * @param time : the time.
   * @param robot : the information of the robot used in this behavior.
   * @param ball : the information of the ball.
   */
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  /**
   * Return the control of the behavior.
   */
  virtual Control control() const;

  /**
   * @see rhoban_ssl::annotations::Annotations
   * The class doesn't draw any annotations.
   * The follower draws the annotations.
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
  
  /**
   * @brief Destructor.
   */
  virtual ~GoCenter();
};
} // namespace beginner
} // namespace robot_behavior
} // namespace rhoban_ssl
