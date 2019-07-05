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
  : RobotBehavior()
  , follower_(Factory::fixedConsignFollower())
  , robot_id_to_steal_(robot_id_to_steal)
  , in_front_of_(false)
  , go_back_(false)
  , final_approach_value_(FINAL_APPROACH_RADIUS_FIRST_VALUE)
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

  const data::Robot& victim = Data::get()->robots[Opponent][robot_id_to_steal_];
  rhoban_geometry::Point victim_position = victim.getMovement().linearPosition(time);
  ContinuousAngle victim_rotation = victim.getMovement().angularPosition(time);

  Vector2d vect_robot_victim = victim_position - robot_position;
  ContinuousAngle target_rotation = vector2angle(vect_robot_victim);

  follower_->avoidRobot(victim.id + ai::Config::NB_OF_ROBOTS_BY_TEAM,
                        true);  //+ ai::Config::NB_OF_ROBOTS_BY_TEAM because id return the id of the bot relative to
                                // its team.

  // differents phases of the behavior:
  double dist_with_victim;
  if (go_back_)
  {
    dist_with_victim = RESET_RADIUS + 0.1;
    if (robot_position.getDist(victim_position) <= APPROACH_PERIMETER)
    {
      follower_->avoidRobot(victim.id + ai::Config::NB_OF_ROBOTS_BY_TEAM, false);
    }
  }
  else if (!in_front_of_)
  {
    dist_with_victim = APPROACH_PERIMETER;
  }
  else
  {
    dist_with_victim = final_approach_value_ + ai::Config::robot_radius;
    final_approach_value_ -= FINAL_APPROACH_DECREASE_SPEED;
    follower_->avoidRobot(victim.id + ai::Config::NB_OF_ROBOTS_BY_TEAM, false);
  }

  target_position = rhoban_geometry::Point(victim_position.x + dist_with_victim * std::cos(victim_rotation.value()),
                                           victim_position.y + dist_with_victim * std::sin(victim_rotation.value()));

  // compute booleans:
  if (robot_position.getDist(target_position) <= ZONE_PRECISION_RADIUS)
  {
    in_front_of_ = true;
  }
  if (robot_position.getDist(victim_position) >= RESET_RADIUS)
  {
    in_front_of_ = false;
    go_back_ = false;
    final_approach_value_ = FINAL_APPROACH_RADIUS_FIRST_VALUE;
  }

  if (infraRed() || robot_position.getDist(victim_position) <= (2 * ai::Config::robot_radius))
  {
    go_back_ = true;
  }

  follower_->avoidTheBall(false);
  annotations_.addCross(target_position);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control BenStealer::control() const
{
  Control ctrl = follower_->control();
  /*
  if (in_front_of_ || go_back_)
  {
    ctrl.spin = true;
  }
  else
  {
    ctrl.spin = false;
  }*/
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
