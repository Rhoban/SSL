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

#ifndef ROBOT_BEHAVIOR__BEGINNER__SEE_BALL__
#define ROBOT_BEHAVIOR__BEGINNER__SEE_BALL__

#include <robot_behavior/robot_behavior.h>
#include <robot_behavior/factory.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace beginner
{
/**
 * @class See Ball
 *
 * @brief Tutorial to show how to see the ball.
 */
class SeeBall : public RobotBehavior
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
   *
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @see Ai::AiData
   */
  SeeBall(Ai::AiData& ai_data);

  /**
   * @brief The robot stay in his position and always turns to the ball.
   *
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   */
  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  /**
   * @see Control
   */
  virtual Control control() const;

  /**
   * @see RhobanSSLAnnotation::Annotations
   * The class don't draw any annotations.
   * The follower draw annotation.
   */
  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  /**
   * @brief Destructor.
   */
  virtual ~SeeBall();
};

};  // namespace beginner
};  // Namespace Robot_behavior
};  // Namespace RhobanSSL

#endif
