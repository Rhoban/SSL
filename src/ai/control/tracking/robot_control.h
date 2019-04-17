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
#ifndef ROBOT_CONTROL_TRACKING_H
#define ROBOT_CONTROL_TRACKING_H

#include <control/control.h>
#include <iostream>
#include <fstream>

namespace rhoban_ssl
{
namespace robot_control
{
namespace tracking
{
/**
 * @class RobotControl
 * @brief The RobotControl class
 * @todo ADD documentation for this file
 */
class RobotControl
{
public:
  RobotControl() : logs("test.txt", std::ios::out | std::ios::trunc)
  {
  }
  virtual ~RobotControl()
  {
    logs.close();
  }

  /**
   * @brief limitedControl
   * @return
   */
  virtual Control getLimitedControl() const = 0;
  virtual void computeLimitedControl() = 0;

  void update(double current_time, const Vector2d& robot_linear_position, const Vector2d& robot_linear_velocity,
              const ContinuousAngle& robot_angular_position);
  void setStatic(bool value);
  bool isStatic() const;

  void initTime(double start_time, double dt);
  double getDt() const;
  double getTime() const;

protected:
  bool static_robot_ = true;
  double start_time_ = -1.0;
  double time_ = 0.0;
  double dt_ = 0.0;

  Vector2d current_linear_position_;
  Vector2d current_linear_velocity_;
  ContinuousAngle current_angular_position_;

  Vector2d linear_position_at_start_;
  ContinuousAngle angular_position_at_start_;

  std::ofstream logs;
private:
  bool init = false;
};

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl

#endif  // ROBOT_CONTROL_H
