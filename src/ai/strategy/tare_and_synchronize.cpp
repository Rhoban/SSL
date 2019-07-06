/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "tare_and_synchronize.h"
#include <robot_behavior/do_nothing.h>
#include <robot_behavior/position_follower.h>

namespace rhoban_ssl
{
namespace strategy
{
const std::string TareAndSynchronize::name = "tare_and_synchronize";

TareAndSynchronize::TareAndSynchronize()
  : Strategy(), halt_behavior_was_assigned_(false), move_behavior_was_assigned_(false), time_is_synchro_(false)
{
}

int TareAndSynchronize::minRobots() const
{
  return 1;
}
int TareAndSynchronize::maxRobots() const
{
  return 1;
}
GoalieNeed TareAndSynchronize::needsGoalie() const
{
  return GoalieNeed::NO;
}

void TareAndSynchronize::start(double time)
{
  DEBUG("START TIME SYNCHRONIZATION");
  halt_behavior_was_assigned_ = false;
  move_behavior_was_assigned_ = false;
  time_is_synchro_ = false;
}

bool TareAndSynchronize::isTaredAndSynchronized() const
{
  return time_is_synchro_;
}

void TareAndSynchronize::update(double time)
{
}

void TareAndSynchronize::stop(double time)
{
  DEBUG("STOP TIME SYNCHRONIZATION");
}

double TareAndSynchronize::getTemporalShiftBetweenVision() const
{
  double propagation_time_of_command = ai_time_associated_to_vision_time_command_ - ai_time_command_;
  double estimated_time_of_command_application = ai_time_command_ + propagation_time_of_command / 2.0;
  double temporal_shift = estimated_time_of_command_application - vision_time_command_;
  return temporal_shift;
}

void TareAndSynchronize::setTemporalShiftBetweenVision()
{
  Data::get()->ai_data.time_shift_with_vision = getTemporalShiftBetweenVision();
}

void TareAndSynchronize::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  const Movement& movement = Data::get()->robots[Ally][robotId(0)].getMovement();
  if (!halt_behavior_was_assigned_)
  {
    assign_behavior(robotId(0), std::shared_ptr<robot_behavior::RobotBehavior>(new robot_behavior::DoNothing()));
    halt_behavior_was_assigned_ = true;
    return;
  }
  if (halt_behavior_was_assigned_ and !move_behavior_was_assigned_)
  {
    if (movement.angularVelocity(movement.lastTime()).abs().value() <= 0.05)
    {
      robot_behavior::PositionFollower* follower = new robot_behavior::PositionFollower(time, dt);
      follower->setFollowingPosition(movement.linearPosition(movement.lastTime()),
                                     movement.angularPosition(movement.lastTime()) + M_PI / 2.0);
      follower->setTranslationPid(ai::Config::p_translation, ai::Config::i_translation, ai::Config::d_translation);
      follower->setOrientationPid(ai::Config::p_orientation, ai::Config::i_orientation, ai::Config::d_orientation);
      follower->setLimits(ai::Config::translation_velocity_limit, ai::Config::rotation_velocity_limit,
                          ai::Config::translation_acceleration_limit, ai::Config::rotation_acceleration_limit);

      ai_time_command_ = time;
      assign_behavior(robotId(0), std::shared_ptr<robot_behavior::RobotBehavior>(follower));
      move_behavior_was_assigned_ = true;
      return;
    }
  }
  if (halt_behavior_was_assigned_ and move_behavior_was_assigned_ and !time_is_synchro_)
  {
    if (movement.angularVelocity(movement.lastTime()).abs().value() >= 0.05)
    {
      vision_time_command_ = movement.getSample().time();
      ai_time_associated_to_vision_time_command_ = time;
      assign_behavior(robotId(0), std::shared_ptr<robot_behavior::RobotBehavior>(new robot_behavior::DoNothing()));
      setTemporalShiftBetweenVision();
      time_is_synchro_ = true;
      return;
    }
  }
}

TareAndSynchronize::~TareAndSynchronize()
{
}

}  // namespace strategy
}  // namespace rhoban_ssl
