/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

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
 * @brief This behavior permits to change the rotation
 * of a robot.
 */
class ChooseRotation : public RobotBehavior
{
private:
  rhoban_ssl::annotations::Annotations annotations_;

  /**
   * @brief Permits to control the robot
   *
   * The consignFollower need to receive a position
   * and an angle (radian).
   *
   * @warning Don't forget to call the ConsignFollower::update
   * method to apply the new orders.
   */
  ConsignFollower* follower_;

public:
  ChooseRotation(ai::AiData& ai_data);
  virtual ~ChooseRotation();

  // RobotBehavior interface
public:
  /**
   * @see rhoban_ssl::robot_behavior::update
   */
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  /**
   * @see rhoban_ssl::robot_behavior::control
   */
  virtual Control control() const;

  /**
   * @see rhoban_ssl::robot_behavior::getAnnotations
   */
  rhoban_ssl::annotations::Annotations getAnnotations() const;
};

};  // namespace beginner
};  // namespace robot_behavior
};  // namespace rhoban_ssl
