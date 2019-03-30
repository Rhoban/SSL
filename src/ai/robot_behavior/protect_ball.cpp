/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#include "protect_ball.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
ProtectBall::ProtectBall(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data))
{
}

void ProtectBall::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible
  // annotations.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);
  // const rhoban_geometry::Point & opponent_goal_point = opponent_goal_center();

  // Vector2d ball_goal_vector = opponent_goal_point - ball_position();
  Vector2d ball_robot_vector = ballPosition() - robot_position;
  // Vector2d target_position;

  // annotations.addCircle( ball_position().x, ball_position().y , radius );
  // annotations.addAnnotations(follower->get_annotations());

  rhoban_geometry::Point target_position = ballPosition();

  double target_rotation = detail::vec2angle(ball_robot_vector);

  follower->avoid_the_ball(true);
  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control ProtectBall::control() const
{
  Control ctrl = follower->control();
  return ctrl;
}

ProtectBall::~ProtectBall()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations ProtectBall::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
