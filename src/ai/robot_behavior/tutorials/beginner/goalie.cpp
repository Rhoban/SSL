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
namespace robot_behavior
{
namespace beginner
{
Goalie::Goalie() : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
}

void Goalie::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  // The goalie moves between the position of the ally's goal center and the position of the ball.
  // The position of the goalie is at 0.5 meters of the goal center.
  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(GlobalDataSingleThread::singleton_.ai_data_.time);
  rhoban_geometry::Point target_position = robot_position;
  Vector2d goal_ball_vector = ballPosition() - allyGoalCenter();
  double dist_goal_ball_vector = goal_ball_vector.norm();
  double target_rotation = 0;

  if (dist_goal_ball_vector != 0)
  {
    goal_ball_vector = goal_ball_vector / dist_goal_ball_vector;

    // Move the robot 0.5 meters from the goal center. The robot will be aligne with the ally
    // goal center and the ball position.
    target_position = allyGoalCenter() + goal_ball_vector * 0.5;
    target_rotation = detail::vec2angle(goal_ball_vector);
  }

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control Goalie::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

Goalie::~Goalie()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Goalie::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace beginner
}  // namespace robot_behavior
}  // namespace rhoban_ssl
