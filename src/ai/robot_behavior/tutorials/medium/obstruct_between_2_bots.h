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
 * @class ObstructBetween2Bots
 * @brief Tutorial to show how to go between two other robots.
 */
class ObstructBetween2Bots : public RobotBehavior
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
   * @brief The target robot 1 to obstruct.
   */
  int target_id_1_;
  /**
   * @brief The target robot 2 to obstruct.
   */
  int target_id_2_;

public:
  /**
   * @brief Constructor.
   * The default value of target ids are set to ids 0 and 1.
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @param target_id_1_ : ID of robot 1.
   * @param target_id_2_ : ID of robot 2.
   * @see ai::AiData
   */
  ObstructBetween2Bots(ai::AiData& ai_data, int target_id_1_ = 0, int target_id_2_ = 1);

  /**
   * @brief At each iteration of this function, the folower order to the robot to go to the middle point on the line
   * that link the two targets.
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
   * @brief target IDs setter.
   * @param target_id_1 : ID 1 to set.
   * @param target_id_2 : ID 2 to set.
   */
  void setRobotIDsToObstruct(int target_id_1, int target_id_2);

  /**
   * @brief target IDs getter.
   * @return a tuple with the two IDs.
   */
  std::tuple<int,int> getRobotIDsToObstruct() const;

  /**
   * @see rhoban_ssl::annotations::Annotations
   * The class don't draw any annotations.
   * The follower draw annotation.
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   * @brief Destructor.
   */
  virtual ~ObstructBetween2Bots();
};

};  // namespace medium
};  // namespace robot_behavior
};  // namespace rhoban_ssl