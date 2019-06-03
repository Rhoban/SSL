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

#include "pvc_robot_follower.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <math/continuous_angle.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
RobotFollower::RobotFollower()
  : RobotBehavior(), robot_to_follow_id_(-1), team_(Ally), follower_(Factory::fixedConsignFollower())
{
}

void RobotFollower::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  const data::Robot& robot_to_follow = getRobot(robot_to_follow_id_, robot_to_follow_team_);
  rhoban_geometry::Point position = robot_to_follow.getMovement().linearPosition(time);
  rhoban_geometry::Point target_position = position + translation_;

  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);
  double target_rotation = detail::vec2angle(ballPosition() - robot_position);

  follower_->avoidTheBall(true);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control RobotFollower::control() const
{
  Control ctrl = follower_->control();
  // ctrl.spin = true; // We active the dribler !
  return ctrl;
}

RobotFollower::~RobotFollower()
{
  delete follower_;
}

void RobotFollower::declareRobotToFollow(int robot_id, const Vector2d& translation, Team team)
{
  robot_to_follow_id_ = robot_id;
  this->translation_ = translation;
  robot_to_follow_team_ = team;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
