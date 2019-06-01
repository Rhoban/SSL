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

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
Defender::Defender() : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
}

void Defender::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(Data::get()->ai_data.time);

  Vector2d ball_goal_vector = Data::get()->field.goalCenter(Ally) - ballPosition();
  double target_rotation = 0.0;

  rhoban_geometry::Point target_position = robot_position;
  if (!ballIsInsideAllyPenaltyArea())
  {
    double dist_ball_goal = ball_goal_vector.norm();
    if (dist_ball_goal > 0)
    {
      ball_goal_vector = ball_goal_vector / dist_ball_goal;
      // Put the robot at 0.5 meters on the ball on the vector opponent_goal and ball.
      target_position = ballPosition() + ball_goal_vector * 0.5;

      // We always look the ball
      Vector2d direction_to_ball = ballPosition() - robot_position;
      target_rotation = detail::vec2angle(direction_to_ball);
    }
  }
  else
  {
    // The ball is inside the penalty area. Don't nothing.
  }

  follower_->avoidTheBall(true);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

bool Defender::ballIsInsideAllyPenaltyArea()
{
  return allyPenaltyArea().is_inside(ballPosition());
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

rhoban_ssl::annotations::Annotations Defender::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}
}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl
