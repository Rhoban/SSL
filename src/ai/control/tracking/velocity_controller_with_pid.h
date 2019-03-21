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

#ifndef __VELOCITY_CONTROLLER_WITH_PID__H__
#define __VELOCITY_CONTROLLER_WITH_PID__H__

#include <math/vector2d.h>
#include <control/tracking/robot_control.h>
#include <control/tracking/pid.h>

namespace rhoban_ssl
{
namespace robot_control
{
namespace tracking
{
/**
 * @brief The VelocityControllerWithPid class
 *
 * @todo ADD DESCRIPTION for all this file
 */
class VelocityControllerWithPid : public RobotControl
{
  // RobotControl interface
public:
  /**
   * @see RobotControl::getLimitedControl
   */
  virtual Control getLimitedControl() const;

  /**
   * @see RobotControl::computeLimitedControl
   */
  virtual void computeLimitedControl();

public:
  /**
   * @brief VelocityControllerWithPid
   */
  VelocityControllerWithPid() = default;

  virtual ~VelocityControllerWithPid() = default;

  /**
   * @brief linearVelocity
   * @param time
   * @return
   */
  virtual Vector2d linearVelocity(double time) const = 0;
  /**
   * @brief linearPosition
   * @param time
   * @return
   */
  virtual Vector2d linearPosition(double time) const = 0;
  /**
   * @brief angularVelocity
   * @param time
   * @return
   */
  virtual ContinuousAngle angularVelocity(double time) const = 0;
  /**
   * @brief angularPosition
   * @param time
   * @return
   */
  virtual ContinuousAngle angularPosition(double time) const = 0;

  /**
   * @brief setLinearVelocityPid
   * @param kp
   * @param ki
   * @param kd
   */
  void setLinearVelocityPid(double kp, double ki, double kd);
  /**
   * @brief setAngularVelocityPid
   * @param kp
   * @param ki
   * @param kd
   */
  void setAngularVelocityPid(double kp, double ki, double kd);

private:
  /**
   * @brief computeNoLimitedLinearVelocity
   * @param robot_position
   * @return
   */
  Vector2d computeNoLimitedLinearVelocity(const Vector2d& robot_position);

  /**
   * @brief computeNoLimitedAngularVelocity
   * @param robot_angular_position
   * @return
   */
  ContinuousAngle computeNoLimitedAngularVelocity(const ContinuousAngle& robot_angular_position);

private:
  Pid linear_velocity_pid_;
  Pid angular_velocity_pid_;

  ContinuousAngle computed_limited_angular_velocity_;
  Vector2d computed_limited_linear_velocity_;
};

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl
#endif
