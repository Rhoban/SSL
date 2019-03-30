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

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace beginner
{
/**
 */
/**
 * @class Goalie
 * @brief Tutorial class to show how to place the robot behind the
 *  ball which aiming the center of ball.
 */
class Goalie : public RobotBehavior
{
private:
  /**
   * @see RhobanSSL::Robot_behavior::ConsignFollower
   */
  ConsignFollower* follower_;
  /**
   * Not use in this package but set in a case of copy.
   * @see RhobanSSLAnnotation::Annotations
   */
  RhobanSSLAnnotation::Annotations annotations_;

public:
  /**
   * @brief Constructor.
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @see Ai::AiData
   */
  Goalie(Ai::AiData& ai_data);

  /**
   * @brief Move the robot 0.5 meters from the goal center.
   * The robot will be align with the ally goal center and the ball position..
   *
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   */
  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  /**
   * @see Control.
   */
  virtual Control control() const;

  /**
   * @see RhobanSSLAnnotation::Annotations
   */
  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  /**
   * @brief Destructor.
   */
  virtual ~Goalie();
};

}  // Namespace beginner
}  // Namespace Robot_behavior
}  // namespace RhobanSSL
