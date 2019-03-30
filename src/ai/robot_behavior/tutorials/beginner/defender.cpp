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
#include "defender.h"
#include <math/vector2d.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace beginner
{
Defender::Defender(Ai::AiData& ai_data) : RobotBehavior(ai_data), follower_(Factory::fixed_consign_follower(ai_data))
{
}

void Defender::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::update_time_and_position(time, robot, ball);
  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.get_movement().linear_position(ai_data.time);

  Vector2d ball_goal_vector = ally_goal_center() - ball_position();
  double target_rotation = detail::vec2angle(ball_goal_vector);

  rhoban_geometry::Point target_position = robot_position;
  if (!ball_is_inside_ally_penalty_area())
  {
    double dist_ball_goal = ball_goal_vector.norm();
    if (dist_ball_goal > 0)
    {
      ball_goal_vector = ball_goal_vector / dist_ball_goal;
      // Put the robot at 0.5 meters on the ball on the vector opponent_goal and ball.
      target_position = ball_position() + ball_goal_vector * 0.5;
    }
  }
  else
  {
    // The ball is inside the penalty area. Don't nothing.
  }

  follower_->avoid_the_ball(true);
  follower_->set_following_position(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

bool Defender::ball_is_inside_ally_penalty_area()
{
  return ally_penalty_area().is_inside(ball_position());
  ;
}

Control Defender::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

Defender::~Defender()
{
  delete follower_;
}

RhobanSSLAnnotation::Annotations Defender::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->get_annotations());
  return annotations;
}
}  // namespace beginner
}  // namespace Robot_behavior
}  // namespace RhobanSSL
