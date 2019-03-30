/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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

#include "goalie.h"
#include <math/vector2d.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace Beginner
{
Goalie::Goalie(Ai::AiData& ai_data) : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data))
{
}

void Goalie::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  RobotBehavior::update_time_and_position(time, robot, ball);

  // The goalie moves between the position of the ally's goal center and the position of the ball.
  // The position of the goalie is at 0.5 meters of the goal center.

  const rhoban_geometry::Point& robot_position = robot.get_movement().linear_position(ai_data.time);
  rhoban_geometry::Point target_position = robot_position;
  Vector2d goal_ball_vector = ball_position() - ally_goal_center();
  double dist_goal_ball_vector = goal_ball_vector.norm();

  if (dist_goal_ball_vector != 0)
  {
    goal_ball_vector = goal_ball_vector / dist_goal_ball_vector;

    // Move the robot 0.5 meters from the goal center. The robot will be aligne with the ally
    // goal center and the ball position.
    target_position = ally_goal_center() + goal_ball_vector * 0.5;
  }

  double target_rotation = detail::vec2angle(goal_ball_vector);

  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Goalie::control() const
{
  Control ctrl = follower->control();
  return ctrl;
}

Goalie::~Goalie()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Goalie::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  annotations.addAnnotations(follower->get_annotations());
  return annotations;
}
}  // namespace Beginner
}  // namespace Robot_behavior
}  // namespace RhobanSSL
