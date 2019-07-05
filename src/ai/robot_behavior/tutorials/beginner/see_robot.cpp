/*
    This file is part of SSL.

    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)

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

#include "see_robot.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace beginner
{
SeeRobot::SeeRobot(uint target_number)
  : RobotBehavior(), target_robot_number(target_number), follower_(Factory::fixedConsignFollower())
{
}

void SeeRobot::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(Data::get()->time.now());

  ContinuousAngle target_rotation = robot.getMovement().angularPosition(Data::get()->time.now());

  // Condition to check if the target robot is not the robot itself.
  // A robot which try to look itself will do nothing.
  if (target_robot_number != robot.id)
  {
    const rhoban_geometry::Point& target_position =
        Data::get()->robots[Ally][target_robot_number].getMovement().linearPosition(Data::get()->time.now());

    Vector2d direction = target_position - robot_position;
    target_rotation = vector2angle(direction);
  }

  follower_->setFollowingPosition(robot_position, target_rotation);

  follower_->avoidTheBall(false);

  follower_->update(time, robot, ball);
}

void SeeRobot::setRobotIdToSee(uint id)
{
  target_robot_number = id;
}

uint SeeRobot::getRobotIdToSee() const
{
  return target_robot_number;
}

Control SeeRobot::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

SeeRobot::~SeeRobot()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations SeeRobot::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace beginner
}  // namespace robot_behavior
}  // namespace rhoban_ssl
