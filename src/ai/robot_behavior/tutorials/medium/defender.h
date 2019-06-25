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

#include "robot_behavior/factory.h"
#include "robot_behavior/robot_behavior.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
/**
 * @class Defender
 * @brief Tutorial shows how to place a robot between the ally's goal and the ball. (Defend situation)
 */
class Defender : public RobotBehavior
{
private:
  /**
   * @see rhoban_ssl::robot_behavior::ConsignFollower
   */
  ConsignFollower* follower_;
  /**
   * Not use in this package but set in a case of copy.
   * @see rhoban_ssl::annotations::Annotations
   */
  rhoban_ssl::annotations::Annotations annotations_;

public:
  /**
   * @brief Constructor.
   * The default value of the target_corner_ is set to left opponent corner.
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @see ai::AiData
   */
  Defender();

  /**
   * @brief Is the ball is inside the penalty area.
   * @return True if the ball is inside the penalty area.
   */
  bool ballIsInsideAllyPenaltyArea();
  /**
   * @brief Put the robot between the ball and the ally goal zone.
   *
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   */
  void update(double time, const data::Robot& robot, const data::Ball& ball);

  /**
   * @see Control.
   */
  virtual Control control() const;

  /**
   * @see rhoban_ssl::annotations::Annotations
   * The class don't draw any annotations.
   * The follower draw annotation.
   */
  rhoban_ssl::annotations::Annotations getAnnotations() const;
  /**
   * @brief Destructor.
   */
  virtual ~Defender();
};

}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl
