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
#include "ben_stealer.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
BenStealer::BenStealer() : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
}

void BenStealer::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  annotations_.clear();

  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);
  ContinuousAngle robot_rotation = robot.getMovement().angularPosition(time);

  rhoban_geometry::Point target_position = robot_position;
  double target_rotation = robot_rotation.value();

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control BenStealer::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

BenStealer::~BenStealer()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations BenStealer::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}
}  // namespace robot_behavior
}  // namespace rhoban_ssl
