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

#include "robot_behavior/factory.h"
#include "robot_behavior/robot_behavior.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace test
{
/**
 * @class KickMeasure
 * @brief Behavior to just kick with a specified power, need to put the ball at the middle of the kicker before apply
 * the behavior.
 */
class KickMeasure : public RobotBehavior
{
private:
  /**
   * @see rhoban_ssl::robot_behavior::ConsignFollower
   */
  ConsignFollower* follower_;
  /**
   * @see rhoban_ssl::annotations::Annotations
   */
  rhoban_ssl::annotations::Annotations annotations_;
  /**
   * @brief Kick power ([0;1])
   */
  double kick_power_;

public:
  /**
   * @brief Constructor.
   * The default value of power is set to 1.0
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @param kick_power : power of the kick, betwen 0 and 1
   * @see ai::AiData
   */
  KickMeasure(ai::AiData& ai_data, double kick_power = 1.0);

  /**
   * @brief This behavior doesn't move the robot, when launched it supposes that the ball is already in front of the
   * robot and then kick the ball with the specified power, the distance between the robot and the ball is conseqently
   * printed.
   *
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   */
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  /**
   * @return the control of the behavior.
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
  virtual ~KickMeasure();
};

};  // namespace test
};  // namespace robot_behavior
};  // namespace rhoban_ssl