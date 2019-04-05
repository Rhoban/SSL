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
namespace medium
{
/**
 * @class FollowRobot
 * @brief Tutorial to show how follow a other robot.
 */
class FollowRobot : public RobotBehavior
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
   * @brief Constant value to define distance from which the robot is close enough the target.
   */
  const double TRACKING_DISTANCE = 0.3;
  /**
   * @brief The target robot to follow.
   */
  int target_id_;

public:
  /**
   * @brief Constructor.
   * The default value of the target_id is set to the id 0.
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @param target_id : ID of robot to follow.
   * @see ai::AiData
   */
  FollowRobot(ai::AiData& ai_data, int target_id = 0);

  /**
   * @brief At each iteration of this function, the robot go to a point on the line between it and the target, at a
   * certain distance from the target. This distance is TRACKING_DISTANCE constant. If the robot is nearest, it doesn't
   * move anymore while the target is still in this perimeter.
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
   * @brief target_id_ setter.
   * @param target_id : ID to set.
   */
  void setRobotIdToFollow(int target_id);

  /**
   * @brief target_id_ getter.
   * @return target_id_.
   */
  int getRobotIdToFollow() const;

  /**
   * @see rhoban_ssl::annotations::Annotations
   * The class don't draw any annotations.
   * The follower draw annotation.
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   * @brief Destructor.
   */
  virtual ~FollowRobot();
};

};  // namespace medium
};  // namespace robot_behavior
};  // namespace rhoban_ssl