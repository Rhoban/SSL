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
namespace Strategy
{
const std::string Tare_and_synchronize::name = "tare_and_synchronize";

Tare_and_synchronize::Tare_and_synchronize(ai::AiData& ai_data)
  : Strategy(ai_data), halt_behavior_was_assigned(false), move_behavior_was_assigned(false), time_is_synchro(false)
{
}

int Tare_and_synchronize::min_robots() const
{
  return 1;
}
int Tare_and_synchronize::max_robots() const
{
  return 1;
}
Goalie_need Tare_and_synchronize::needs_goalie() const
{
  return Goalie_need::NO;
}

void Tare_and_synchronize::start(double time)
{
  DEBUG("START TIME SYNCHRONIZATION");
  halt_behavior_was_assigned = false;
  move_behavior_was_assigned = false;
  time_is_synchro = false;
}

bool Tare_and_synchronize::is_tared_and_synchronized() const
{
  return time_is_synchro;
}

void Tare_and_synchronize::update(double time)
{
}

void Tare_and_synchronize::stop(double time)
{
  DEBUG("STOP TIME SYNCHRONIZATION");
}

double Tare_and_synchronize::get_temporal_shift_between_vision() const
{
  double propagation_time_of_command = ai_time_associated_to_vision_time_command - ai_time_command;
  double estimated_time_of_command_application = ai_time_command + propagation_time_of_command / 2.0;
  double temporal_shift = estimated_time_of_command_application - vision_time_command;
  return temporal_shift;
}

void Tare_and_synchronize::set_temporal_shift_between_vision()
{
  ai_data.time_shift_with_vision = get_temporal_shift_between_vision();
}

void Tare_and_synchronize::assign_behavior_to_robots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  const Movement& movement = ai_data.robots[Vision::Ally][robot_id(0)].getMovement();
  if (!halt_behavior_was_assigned)
  {
    assign_behavior(robot_id(0),
                    std::shared_ptr<Robot_behavior::RobotBehavior>(new Robot_behavior::DoNothing(ai_data)));
    halt_behavior_was_assigned = true;
    return;
  }
  if (halt_behavior_was_assigned and !move_behavior_was_assigned)
  {
    if (movement.angular_velocity(movement.last_time()).abs().value() <= 0.05)
    {
      Robot_behavior::PositionFollower* follower = new Robot_behavior::PositionFollower(ai_data, time, dt);
      follower->set_following_position(movement.linear_position(movement.last_time()),
                                       movement.angular_position(movement.last_time()) + M_PI / 2.0);
      follower->set_translation_pid(ai_data.constants.p_translation, ai_data.constants.i_translation,
                                    ai_data.constants.d_translation);
      follower->set_orientation_pid(ai_data.constants.p_orientation, ai_data.constants.i_orientation,
                                    ai_data.constants.d_orientation);
      follower->set_limits(ai_data.constants.translation_velocity_limit, ai_data.constants.rotation_velocity_limit,
                           ai_data.constants.translation_acceleration_limit,
                           ai_data.constants.rotation_acceleration_limit);

      ai_time_command = time;
      assign_behavior(robot_id(0), std::shared_ptr<Robot_behavior::RobotBehavior>(follower));
      move_behavior_was_assigned = true;
      return;
    }
  }
  if (halt_behavior_was_assigned and move_behavior_was_assigned and !time_is_synchro)
  {
    if (movement.angular_velocity(movement.last_time()).abs().value() >= 0.05)
    {
      vision_time_command = movement.get_sample().time();
      ai_time_associated_to_vision_time_command = time;
      assign_behavior(robot_id(0),
                      std::shared_ptr<Robot_behavior::RobotBehavior>(new Robot_behavior::DoNothing(ai_data)));
      set_temporal_shift_between_vision();
      time_is_synchro = true;
      return;
    }
  }
}

Tare_and_synchronize::~Tare_and_synchronize()
{
}

}  // namespace Strategy
}  // namespace rhoban_ssl
