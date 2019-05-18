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

#include "kick_to_xy.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
KickToXY::KickToXY(ai::AiData& ai_data, rhoban_geometry::Point target_point)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data))

{
  target_point_ = target_point;
}

void KickToXY::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update position from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);

  Vector2d ball_target_vector = target_point_ - ballPosition();
  Vector2d ball_robot_vector = robot_position - ballPosition();
  ball_target_vector = normalized(ball_target_vector);
  ball_robot_vector = normalized(ball_robot_vector);

  double target_radius_from_ball;
  double scalar_ball_robot = -scalarProduct(ball_robot_vector, ball_target_vector);

  if (scalar_ball_robot < 0)  // if the robot is behind the ball
  {
    follower_->avoidTheBall(true);
    target_radius_from_ball = 1.5;
  }
  else  // if the robot need to backward to go behind the ball
  {
    follower_->avoidTheBall(false);
    target_radius_from_ball = 1 / (2 * (scalar_ball_robot - 1.2)) + 2;
  }
  rhoban_geometry::Point follow_position = ballPosition() - ball_target_vector * target_radius_from_ball;
  double follow_rotation = detail::vec2angle(ball_target_vector);

  follower_->setFollowingPosition(follow_position, follow_rotation);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control KickToXY::control() const
{
  Control ctrl = follower_->control();
  ctrl.kickPower = 1.0;
  ctrl.kick = true;
  ctrl.charge = true;
  return ctrl;
}

KickToXY::~KickToXY()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations KickToXY::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl