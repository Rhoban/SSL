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
namespace medium
{
/**
 * @class Lob
 * @brief Tutorial to show how to use the lob.
 *
 * Go in the position of the ball and make a lob.
 */
class Lob : public RobotBehavior
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
  /**
   * A safety margin for the minimal distance to activate the kicker for the Lob.
   * Set at 0.1 here.
   */
  double safety_margin_;
  /**
   * Minimal distance to activate the kicker for the Lob.
   * Set at ball_radius + robot_radius + safety_margin_.
   */
  double dist_minimal_to_lob_;

public:
  /**
   * @brief Constructor.
   * @param ai_data : The Robot Behavior needs the data of the AI.
   * @see ai::AiData
   */
  Lob(ai::AiData& ai_data);

  /**
   * We use parameters to update the time and the position before to do anything.
   * @param time : The time.
   * @param robot : The information for the robot selected in the behavior.
   * @param ball : The information of the ball.
   */
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  /**
   * @see Control.
   */
  virtual Control control() const;

  /**
   * @see  rhoban_ssl::annotations::Annotations.
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   * @brief Destructor.
   */
  virtual ~Lob();
};
}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl
