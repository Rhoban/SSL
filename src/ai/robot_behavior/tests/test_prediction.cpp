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

#include "test_prediction.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
TestPrediction::TestPrediction(rhoban_geometry::Point point, double reach_radius, bool angle)
  : RobotBehavior()
  , follower_(Factory::fixedConsignFollower())
  , target_point_(point)
  , reached_(false)
  , reach_radius_(reach_radius)
  , angle_(angle)

{
}

void TestPrediction::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  const rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);

  if (robot_position.getDist(target_point_) <= reach_radius_)
  {
    reached_ = true;
  }
  rhoban_geometry::Point position_follower = target_point_;

  if (!reached_)
  {
    Vector2d vect_robot_target = target_point_ - robot_position;
    rotation_follower_ = vector2angle(vect_robot_target).value();
  }
  if (angle_)
  {
    rotation_follower_ = 0;
  }
  annotations_.clear();
  annotations_.addCross(Data::get()->ball.getMovement().linearPosition(time +0.5),"red",false);
  annotations_.addCross(Data::get()->ball.getMovement().linearPosition(time +1),"red",false);
  annotations_.addCross(Data::get()->ball.getMovement().linearPosition(time +1.5),"red",false);
  annotations_.addCross(Data::get()->ball.getMovement().linearPosition(time +2),"red",false);
  annotations_.addCross(Data::get()->ball.getMovement().linearPosition(time +2.5),"red",false);
  annotations_.addCross(Data::get()->ball.getMovement().linearPosition(time +3),"red",false);

  annotations_.addCross(robot.getMovement().linearPosition(time + 1), "green", false);
  annotations_.addCross(robot.getMovement().linearPosition(time + 2), "green", false);
  annotations_.addCross(robot.getMovement().linearPosition(time + 3), "green", false);
  
  follower_->setFollowingPosition(position_follower, rotation_follower_);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

void TestPrediction::setPoint(rhoban_geometry::Point point)
{
  target_point_ = point;
  reached_ = false;
}

rhoban_geometry::Point TestPrediction::getPoint() const
{
  return target_point_;
}

void TestPrediction::setReachRadius(double radius)
{
  reach_radius_ = radius;
}

double TestPrediction::getReachRadius() const
{
  return reach_radius_;
}

bool TestPrediction::isReached()
{
  return reached_;
}

void TestPrediction::dribbler(const bool is_active)
{
  dribbler_is_active_ = is_active;
}

Control TestPrediction::control() const
{
  Control ctrl = follower_->control();
  if (dribbler_is_active_)
  {
    ctrl.spin = true;
  }
  else
  {
    ctrl.spin = false;
  }
  return ctrl;
}

TestPrediction::~TestPrediction()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations TestPrediction::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
