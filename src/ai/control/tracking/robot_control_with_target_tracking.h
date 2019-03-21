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

#ifndef __ROBOT_CONTROL_WITH_TARGET_TRACKING__H__
#define __ROBOT_CONTROL_WITH_TARGET_TRACKING__H__

#include <control/tracking/velocity_controller_with_pid.h>

#include <physic/Movement.h>
#include <physic/movement_with_no_prediction.h>
#include <rhoban_geometry/point.h>

namespace rhoban_ssl
{
namespace robot_control
{
namespace tracking
{
/**
 * @brief The RobotControlWithTargetTracking class
 * @todo add documentation for this file
 */
class RobotControlWithTargetTracking : public VelocityControllerWithPid
{
public:
  RobotControlWithTargetTracking() = default;
  ~RobotControlWithTargetTracking() = default;

  Vector2d setLinearVelocityOfTheTarget(const Vector2d& linear_velocity_of_the_target);
  Vector2d setLinearPositionOfTheTarget(const rhoban_geometry::Point& linear_position_of_the_target);

  // VelocityControllerWithPid interface
public:
  virtual ContinuousAngle angularVelocity(double time) const;
  virtual ContinuousAngle angularPosition(double time) const;

protected:
  Vector2d linear_velocity_of_the_target_;
  Vector2d linear_position_of_the_target_;
};

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl
#endif
