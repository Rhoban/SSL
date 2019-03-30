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

namespace rhoban_ssl
{
namespace Robot_behavior
{
Begginer_goalie::Begginer_goalie(Ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data))
{
}

void Begginer_goalie::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  RobotBehavior::update_time_and_position(time, robot, ball);

  const rhoban_geometry::Point& robot_position = robot.get_movement().linear_position(ai_data.time);

  Vector2d ball_goal_vector = ally_goal_center() - ball_position();
  ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();

  // Put the robot at 0.5 meters on the ball on the vector opponent_goal and ball.
  rhoban_geometry::Point target_position = ally_goal_center() - ball_goal_vector * 0.5;
  double target_rotation = detail::vec2angle(ball_goal_vector);

  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Begginer_goalie::control() const
{
  Control ctrl = follower->control();
  return ctrl;
}

Begginer_goalie::~Begginer_goalie()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Begginer_goalie::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  annotations.addAnnotations(follower->get_annotations());
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
