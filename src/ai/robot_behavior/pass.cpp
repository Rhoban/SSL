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

#include "pass.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
Pass::Pass(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , robot_to_pass_id_(-1)
  , robot_to_pass_team_(vision::Ally)
  , follower_(Factory::fixedConsignFollower(ai_data))
{
}

void Pass::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible
  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);
  const ai::Robot& robot_to_pass = getRobot(robot_to_pass_id_, robot_to_pass_team_);
  rhoban_geometry::Point position_robot_to_pass = robot_to_pass.getMovement().linearPosition(time);
  // rhoban_geometry::Point target_position = ball_position();

  // Striker:
  // rhoban_geometry::Point opponent_goal_point = opponent_goal_center();
  Vector2d ball_robot_to_pass_vector = position_robot_to_pass - ballPosition();
  Vector2d ball_robot_vector = robot_position - ballPosition();
  ball_robot_to_pass_vector = ball_robot_to_pass_vector / ball_robot_to_pass_vector.norm();
  ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();
  double target_radius_from_ball;
  double scalar_ball_robot = -scalarProduct(ball_robot_vector, ball_robot_to_pass_vector);

  if (scalar_ball_robot < 0)
  {
    follower_->avoidTheBall(true);
    target_radius_from_ball = 1.5;
  }
  else
  {
    follower_->avoidTheBall(false);
    target_radius_from_ball = 1 / (2 * (scalar_ball_robot - 1.2)) + 2;
  }
  rhoban_geometry::Point target_position = ballPosition() - ball_robot_to_pass_vector * target_radius_from_ball;
  double target_rotation = detail::vec2angle(ball_robot_to_pass_vector);

  // follower->avoid_the_ball(false);
  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control Pass::control() const
{
  Control ctrl = follower_->control();
  ctrl.charge = true;
  ctrl.kickPower = 0.5;
  ctrl.kick = true;
  return ctrl;
}

void Pass::declareRobotToPass(int robot_id, vision::Team team)
{
  robot_to_pass_id_ = robot_id;
  robot_to_pass_team_ = team;
}

Pass::~Pass()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Pass::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
