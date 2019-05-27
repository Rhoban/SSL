/*
    This file is part of SSL.

    Copyright 2018 TO COMPLETE

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

#include "striker.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
Striker::Striker(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , use_custom_vector_(false)
  , striking_point_(opponentGoalCenter())
  , follower_(Factory::fixedConsignFollower(ai_data))
{
}

void Striker::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);

  // rhoban_geometry::Point opponent_goal_point = opponent_goal_center();
  // rhoban_geometry::Point left_post_position = rhoban_geometry::Point( ai_data.field.fieldLength / 2.0,
  // ai_data.field.goalWidth / 2.0 ); rhoban_geometry::Point right_post_position = rhoban_geometry::Point(
  // ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0 );

  Vector2d ball_goal_vector = striking_point_ - ballPosition();
  Vector2d ball_robot_vector = robot_position - ballPosition();
  // Vector2d ball_l_post_vector = left_post_position - ball_position();
  // Vector2d ball_r_post_vector = right_post_position - ball_position();
  double dist_ball_robot = ball_robot_vector.norm();

  ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
  ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();
  // ball_l_post_vector = ball_l_post_vector / ball_l_post_vector.norm();
  // ball_r_post_vector = ball_r_post_vector / ball_r_post_vector.norm();

  // double goal_visible_angle = scalar_product( ball_l_post_vector , ball_r_post_vector );

  double target_radius_from_ball;
  double scalar_ball_robot = -scalarProduct(ball_robot_vector, ball_goal_vector);

  if (scalar_ball_robot < 0)
  {
    follower_->avoidTheBall(true);
    target_radius_from_ball = 0.4;
  }
  else
  {
    follower_->avoidTheBall(false);
    // target_radius_from_ball = 1 / ( 2*(scalar_ball_robot - 1.2) ) + 2;
    // target_radius_from_ball = 1.0 / ( 4.0*(scalar_ball_robot - 1.4) ) + 0.55;
    target_radius_from_ball = 1.0 / (24.0 * (scalar_ball_robot - 1.04)) + 0.44;

    // if ( infra_red() || Vector2d(robot_position - ball_position()).norm() > 0.4 ) {
    //    follower->avoid_opponent(true);
    //}

    // target_radius_from_ball = 1.0 /(2.0 *(scalar_ball_robot - (goal_visible_angle + 0.21))) + 1.0 /
    // (goal_visible_angle + 0.21) + 2.0 * goal_visible_angle - 0.8;

    // if ( scalar_ball_robot < goal_visible_angle) {
    //    target_radius_from_ball = 1.0;
    //} else {
    //    target_radius_from_ball = -0.3;
    //}
    //
    if (dist_ball_robot < 0.4)
    {
      follower_->avoidOpponent(false);
    }
  }
  if (dist_ball_robot > 0.4)
  {
    follower_->avoidOpponent(true);
  }

  rhoban_geometry::Point target_position = ballPosition() - ball_goal_vector * (target_radius_from_ball);
  double target_rotation = detail::vec2angle(ball_goal_vector);

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control Striker::control() const
{
  Control ctrl = follower_->control();
  ctrl.charge = true;
  ctrl.kick = true;
  return ctrl;
}

void Striker::declarePointToStrike(rhoban_geometry::Point point)
{
  striking_point_ = point;
}

Striker::~Striker()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Striker::getAnnotations() const
{
  return follower_->getAnnotations();
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
