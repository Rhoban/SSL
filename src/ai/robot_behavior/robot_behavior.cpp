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

#include "robot_behavior.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace detail
{
double vec2angle(Vector2d direction)
{
  double norm = direction.norm();
  if (norm == 0.0)
    return 0.0;
  direction /= norm;
  double res = std::acos(direction[0]);
  if (direction[1] <= 0)
    return -res;
  return res;
}

}  // namespace detail

RobotBehavior::RobotBehavior() : GameInformations(), robot_ptr_(nullptr), birthday_(-1.0){};

double RobotBehavior::age() const
{
  return last_update_ - birthday_;
};
bool RobotBehavior::isBorn() const
{
  return birthday_ > 0;
};
void RobotBehavior::setBirthday(double birthday)
{
  assert(birthday > 0);
  birthday_ = birthday;
};

void RobotBehavior::updateTimeAndPosition(double time, const data::Robot& robot, const data::Ball& ball)
{
  robot_ptr_ = &robot;
  assert((robot.id >= 0) && (robot.id < ai::Config::NB_OF_ROBOTS_BY_TEAM));
  last_update_ = time;
  robot_linear_position_ = Vector2d(robot.getMovement().linearPosition(time));
  robot_angular_position_ = robot.getMovement().angularPosition(time);
  robot_linear_velocity_ = robot.getMovement().linearVelocity(time);
  robot_angular_velocity_ = robot.getMovement().angularVelocity(time);
};

const data::Robot& RobotBehavior::robot() const
{
#ifndef NDEBUG
  if (not(robot_ptr_))
  {
    DEBUG("if you have this assert, that mean you have probably call a function using robot() before an update that "
          "lunch an update_time_and_position()");
  }
#endif
  assert(robot_ptr_);
  return *robot_ptr_;
}

rhoban_geometry::Point RobotBehavior::linearPosition() const
{
  return robot().getMovement().linearPosition(Data::get()->ai_data.time);
}

ContinuousAngle RobotBehavior::angularPosition() const
{
  return robot().getMovement().angularPosition(Data::get()->ai_data.time);
}

bool RobotBehavior::isGoalie() const
{
  return robot().is_goalie;
}

rhoban_ssl::annotations::Annotations RobotBehavior::getAnnotations() const
{
  return rhoban_ssl::annotations::Annotations();
}

bool RobotBehavior::infraRed() const
{
  return robot().infraRed();
}

///////////////////////////////////////////////////////////////////////////////

RobotBehaviorTask::RobotBehaviorTask(uint robot_number, robot_behavior::RobotBehavior* robot_behavior)
  : robot_behavior_(robot_behavior), robot_number_(robot_number)
{
  auto& final_control = Data::get()->shared_data.final_control_for_robots[robot_number];
  final_control.is_manually_controled_by_viewer = false;
}

RobotBehaviorTask::~RobotBehaviorTask()
{
  delete robot_behavior_;
}

bool RobotBehaviorTask::runTask()
{
  data::Robot& robot = Data::get()->robots[Ally][robot_number_];
  data::Ball& ball = Data::get()->ball;

  double time = Data::get()->ai_data.time;
  double dt = Data::get()->ai_data.dt;
  //  DEBUG("t : " << robot_behavior_-><< std::endl << "time : " << time << std::endl << "dt : " << dt);

  robot_behavior_->update(Data::get()->time.now(), robot, ball);
  Control& ctrl = Data::get()->shared_data.final_control_for_robots[robot_number_].control;

  ctrl = robot_behavior_->control();
  DEBUG("RB: " << ctrl.linear_velocity.norm());
  ctrl.changeToRelativeControl(robot.getMovement().angularPosition(time), dt);
  DEBUG("RB2: " << ctrl.linear_velocity.norm());
  return true;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
