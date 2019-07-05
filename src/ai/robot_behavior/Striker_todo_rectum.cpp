/*
    This file is part of SSL.

    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)
    Copyright 2019 Jérémy Bezamat (jeremy.bezamat@gmail.com)

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

#include "Striker_todo_rectum.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
Striker_todo_rectum::Striker_todo_rectum(rhoban_geometry::Point point)
  : RobotBehavior()
  , follower_(Factory::fixedConsignFollower())
  , target_point_(point)
  , rotated_(false)
{
}

void Striker_todo_rectum::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  const rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);

//   if (robot_position.getDist(Data::get()->ball.getMovement().linearPosition(time)) <= reach_radius_)
    if(Data::get()->robots[Ally][robot.id].infraRed() != true)
  {
    rotated_ = false;
    rhoban_geometry::Point ball_pose = Data::get()->ball.getMovement().linearPosition(time);
    rhoban_geometry::Point position_follower = ball_pose;
    Vector2d vect_robot_target = ball_pose - robot_position;
    ContinuousAngle rotation_follower = vector2angle(vect_robot_target);
    follower_->setFollowingPosition(position_follower, rotation_follower);
    follower_->avoidTheBall(false);
    follower_->update(time, robot, ball);
  }
    else
  {
    rhoban_geometry::Point ball_pose = Data::get()->ball.getMovement().linearPosition(time);
    rhoban_geometry::Point position_follower = robot_position;
    Vector2d vect_robot_target = target_point_ - robot_position;
    ContinuousAngle rotation_follower = vector2angle(vect_robot_target);
    follower_->setFollowingPosition(position_follower, rotation_follower);
    follower_->avoidTheBall(false);
    follower_->update(time, robot, ball);
    if ((rotation_follower - robot.getMovement().angularPosition(time)) < 0.05 ){
      rotated_ = true;
    }
  }

}

Control Striker_todo_rectum::control() const
{
  Control ctrl = follower_->control();
  ctrl.charge = true;
  ctrl.kick = false;
  if(rotated_){
    ctrl.kick = true;
  }
  return ctrl;
}


void Striker_todo_rectum::setPoint(rhoban_geometry::Point point)
{
  target_point_ = point;
  reached_ = false;
}

rhoban_geometry::Point Striker_todo_rectum::getPoint() const
{
  return target_point_;
}

bool Striker_todo_rectum::isReached()
{
  return reached_;
}

Striker_todo_rectum::~Striker_todo_rectum()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Striker_todo_rectum::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
