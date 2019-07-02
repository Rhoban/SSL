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
BenStealer::BenStealer(uint robot_id_to_steal)
  : RobotBehavior(), follower_(Factory::fixedConsignFollower()), robot_id_to_steal_(robot_id_to_steal)
{
}

void BenStealer::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  annotations_.clear();

  rhoban_geometry::Point robot_position = linearPosition();
  // ContinuousAngle robot_rotation = angularPosition();

  rhoban_geometry::Point target_position = robot_position;

  // this behavior supposes that the victim moves forward.
  const data::Robot& victim = Data::get()->robots[Opponent][robot_id_to_steal_];
  rhoban_geometry::Point victim_position = victim.getMovement().linearPosition(time);
  ContinuousAngle victim_rotation = victim.getMovement().angularPosition(time);

  target_position = rhoban_geometry::Point(victim_position.x + APPROACH_PERIMETER * std::cos(victim_rotation.value()),
                                           victim_position.y + APPROACH_PERIMETER * std::sin(victim_rotation.value()));

  Vector2d vect_robot_victim = victim_position - robot_position;
  ContinuousAngle target_rotation = vector2angle(vect_robot_victim);

  follower_->avoidTheBall(false);
  annotations_.addCross(target_position);
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

void BenStealer::setRobotIdToSteal(uint id)  // id of opponent
{
  robot_id_to_steal_ = id;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
