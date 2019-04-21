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

#include "robot_control_with_target_tracking.h"

namespace rhoban_ssl
{
namespace robot_control
{
namespace tracking
{
/**
 * @brief The RobotControlWithTargetTrackingAndPositionFollowing class
 * @todo add documentation for this file
 */
class RobotControlWithTargetTrackingAndPositionFollowing : public RobotControlWithTargetTracking
{
public:
  RobotControlWithTargetTrackingAndPositionFollowing() = default;

  virtual void setGoal(const Vector2d& robot_destination, const rhoban_geometry::Point& linear_position_of_the_target,
                       const Vector2d& linear_velocity_of_the_target);

  //  void setGoal(const Vector2d& robot_destination, const RhobanSSL::Ai::Object& targeted_object);

  // VelocityControllerWithPid interface
public:
  /**
   * @brief Linear velocity instruction
   *
   * In the current implementation, the linear velocity instruction is constant.
   *
   * @todo Change the velocity instruction according to the distance between the
   * robot and its destination
   *
   * @return a const linear velocity
   */
  virtual Vector2d linearVelocity(double) const;
  virtual Vector2d linearPosition(double time) const;

protected:
  Vector2d robot_destination_;
};

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl
