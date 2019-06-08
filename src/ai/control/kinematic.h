/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.mlr@live.fr)

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

#include <config.h>
#include <execution_manager.h>

namespace rhoban_ssl
{
namespace control
{
class Kinematic
{
public:
  Kinematic();

  struct WheelsSpeed
  {
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
  };

  /**
   * @brief Computes the kinematics for the robot.
   * X and Y speed are [m/s], theta speed is [rad/s]
   * It returns the wheel speed in [turn/s]
   */
  WheelsSpeed compute(double x, double y, double theta) const;

private:
  bool init_;
  double robot_radius_;
  double wheel_radius_;

  double front_left_x_;
  double front_left_y_;
  double front_right_x_;
  double front_right_y_;
  double rear_left_x_;
  double rear_left_y_;
  double rear_right_x_;
  double rear_right_y_;
};

/**
 * @brief The WarningMaximumVelocity task prints a warning if we reach the velocity
 * limit of the wheel of the robot.
 *
 * The limit is defined in the config.json file and depends of the vectorial control
 * implementation in the firmware.
 */
class WarningMaximumVelocity : public Task {

private:
  Kinematic kinematic_;

  // Task interface
public:
  bool runTask();
};

}  // namespace control
}  // namespace rhoban_ssl

