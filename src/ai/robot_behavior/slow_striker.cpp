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

#include "slow_striker.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
SlowStriker::SlowStriker(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , robot_to_pass_id(-1)
  , robot_to_pass_team(Vision::Team::Ally)
  , follower(Factory::fixed_consign_follower(ai_data))
{
  tempo = 0.0;
}

void SlowStriker::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  const rhoban_geometry::Point& robot_position = robot.getMovement().linear_position(ai_data.time);

  if (robot_to_pass_id != -1)
  {  // if point_to_pass wasn't declare and robot_to_pass_id was.
    const ai::Robot& robot_to_pass = getRobot(robot_to_pass_id, robot_to_pass_team);
    striking_point = robot_to_pass.getMovement().linear_position(time);
  }

  Vector2d ball_striking_vector = striking_point - ballPosition();
  Vector2d ball_robot_vector = robot_position - ballPosition();
  double dist_ball_robot = ball_robot_vector.norm();

  ball_striking_vector = ball_striking_vector / ball_striking_vector.norm();
  ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();

  double target_radius_from_ball;
  double scalar_ball_robot = -scalar_product(ball_robot_vector, ball_striking_vector);
  if (tempo == 0.0)
  {
    target_radius_from_ball = 0.3;
  }

  if (scalar_ball_robot < 0)
  {
    follower->avoid_the_ball(true);
  }
  else
  {
    follower->avoid_the_ball(false);

    if (dist_ball_robot < target_radius_from_ball)
    {
      follower->avoid_opponent(false);
    }
  }
  // TODO Add hysteresis
  if (dist_ball_robot > target_radius_from_ball)
  {
    follower->avoid_opponent(true);
  }

  rhoban_geometry::Point target_position = ballPosition() - ball_striking_vector * (target_radius_from_ball);
  double target_rotation = detail::vec2angle(ball_striking_vector);

  double position_margin = 0.05;
  double waiting_time = 3.0;

  if ((Vector2d(target_position - robot_position).norm() < position_margin) && (tempo == 0.0))
  {
    tempo = time;
  }

  // if( Vector2d(target_position - robot_position).norm() > position_margin ) {
  //     tempo = 0.0;
  // }

  if ((Vector2d(target_position - robot_position).norm() < position_margin) && (tempo != 0.0))
  {
    if (std::abs(time - tempo) >= waiting_time)
    {
      target_radius_from_ball = -0.5;
    }
  }
  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control SlowStriker::control() const
{
  Control ctrl = follower->control();
  if (robot_to_pass_id != -1)
  {  // if point_to_pass wasn't declare and robot_to_pass_id was.
    ctrl.kickPower = 0.5;
  }
  else
  {
    ctrl.kickPower = 1;
  }
  ctrl.charge = true;
  ctrl.kick = true;
  return ctrl;
}

void SlowStriker::declare_point_to_strik(rhoban_geometry::Point point)
{
  striking_point = point;
}

void SlowStriker::declare_robot_to_pass(int robot_id, Vision::Team team)
{
  robot_to_pass_id = robot_id;
  robot_to_pass_team = team;
}

SlowStriker::~SlowStriker()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations SlowStriker::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
