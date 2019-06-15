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
#include "test_time_synchronizer.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
TestTimeSynchronizer::TestTimeSynchronizer()
  : follower_(Factory::fixedConsignFollower())
  , time_shift_with_vision_(0.0)
  , initial_ai_time_command_(0.0)
  , first_update_done_(false)
  , time_shift_computed_(false)
{
}

TestTimeSynchronizer::~TestTimeSynchronizer()
{
  delete follower_;
}

bool TestTimeSynchronizer::timeShiftComputed()
{
  return time_shift_computed_;
}

double TestTimeSynchronizer::getComputedTimeShift()
{
  return time_shift_with_vision_;
}

void TestTimeSynchronizer::recomputeTimeShift()
{
  time_shift_with_vision_ = 0.0;
  initial_ai_time_command_ = 0.0;
  first_update_done_ = false;
  time_shift_computed_ = false;
}

void TestTimeSynchronizer::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  const Movement& movement = robot.getMovement();

  bool is_robot_moving = movement.angularVelocity(movement.lastTime()).abs().value() > 0.05;
  if (!first_update_done_ && !is_robot_moving)
  {
    follower_->setFollowingPosition(movement.linearPosition(movement.lastTime()),
                                    movement.angularPosition(movement.lastTime()) + M_PI / 2);
    initial_ai_time_command_ = time;
    first_update_done_ = true;
  }

  if (!time_shift_computed_ && first_update_done_)
  {
    if (is_robot_moving)
    {
      double vision_time_command = movement.getSample().time();
      double ai_time_associated_to_vision_command = time;

      double propagation_time_of_command = ai_time_associated_to_vision_command - initial_ai_time_command_;
      double estimated_time_of_command_application = initial_ai_time_command_ + propagation_time_of_command / 2.0;

      time_shift_with_vision_ = estimated_time_of_command_application - vision_time_command;

      DEBUG("TimeShift test: " << time_shift_with_vision_);
      DEBUG("Current timeShift : " << Data::get()->ai_data.time_shift_with_vision);

      time_shift_computed_ = true;
    }
  }

  follower_->update(time, robot, ball);
}

Control TestTimeSynchronizer::control() const
{
  Control ctrl;

  if (first_update_done_ && !time_shift_computed_)
  {
    ctrl = follower_->control();
  }
  return ctrl;
}

}  // namespace tests
}  // namespace robot_behavior
}  // namespace rhoban_ssl
