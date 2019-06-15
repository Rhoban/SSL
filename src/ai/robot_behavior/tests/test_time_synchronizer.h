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

#include <robot_behavior/robot_behavior.h>
#include <robot_behavior/factory.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
class TestTimeSynchronizer : public RobotBehavior
{
public:
  TestTimeSynchronizer();
  ~TestTimeSynchronizer();
  /**
   * @brief
   * The behavior can't compute instantly the time shift with vision.
   * So you must call this method before get the computed value.
   *
   * @return Returns true if the time shift with vision was compute.
   */
  bool timeShiftComputed();

  /**
   * @brief Returns the computed time shift with vision.
   *
   * You must check if the time shift is computed before call this method.
   * @return the time shift with vision
   */
  double getComputedTimeShift();

  /**
   * @brief Remake the behavior to compute the time shift with the vision
   *
   * By default, the bevavior computes the time shift only once but
   * if a strategy wants to recompute the time_shift, it can call this method.
   */
  void recomputeTimeShift();

  // RobotBehavior interface
public:
  void update(double time, const data::Robot &robot, const data::Ball &ball);
  Control control() const;
private:
  ConsignFollower* follower_;
  double time_shift_with_vision_;
  double initial_ai_time_command_;

  bool first_update_done_;
  bool time_shift_computed_;
};
}
}  // namespace robot_behavior
}  // namespace rhoban_ssl
