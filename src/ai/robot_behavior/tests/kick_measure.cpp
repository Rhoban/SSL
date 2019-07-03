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

#include "kick_measure.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace test
{
KickMeasure::KickMeasure(double kick_power) : RobotBehavior(), follower_(Factory::fixedConsignFollower())

{
  kick_power_ = kick_power;
}

void KickMeasure::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update position from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point follow_position = robot.getMovement().linearPosition(time);
  ContinuousAngle follow_rotation = robot.getMovement().angularPosition(time);

  // double distance = follow_position.getDist(ballPosition());

  // Display of distance:
  // annotations_.addText(std::to_string(distance), ballPosition().x - 0.25, ballPosition().y - 0.25, "white");
  // DEBUG(distance);

  follower_->setFollowingPosition(follow_position, follow_rotation);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control KickMeasure::control() const
{
  Control ctrl = follower_->control();
  ctrl.kick_power = kick_power_;
  ctrl.kick = true;
  ctrl.charge = true;
  return ctrl;
}

KickMeasure::~KickMeasure()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations KickMeasure::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace test
}  // namespace robot_behavior
}  // namespace rhoban_ssl