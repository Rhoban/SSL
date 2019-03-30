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

#include "degageur.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
Degageur::Degageur(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , point_to_pass(66, 66)
  , robot_to_pass_id(-1)
  , robot_to_pass_team(vision::Team::Ally)
  , needKick(false)
  , follower(Factory::fixed_consign_follower(ai_data))
{
}

void Degageur::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  // //TODO: Viser un autre robot
  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);

  //    if ((point_to_pass == rhoban_geometry::Point(66,66)) && (robot_to_pass_id == -1)) {
  // default will be the closest ally robot from the opponent goal center
  robot_to_pass_id = GameInformations::getShirtNumberOfClosestRobot(vision::Team::Ally, opponentGoalCenter());
  //    }

  if (robot_to_pass_id != -1)
  {  // if point_to_pass wasn't declare and robot_to_pass_id was.
    const ai::Robot& robot_to_pass = getRobot(robot_to_pass_id, robot_to_pass_team);
    point_to_pass = robot_to_pass.getMovement().linearPosition(time);
  }

  std::vector<int> robot_in_line = GameInformations::getRobotInLine(robot_position, point_to_pass);

  if (robot_position.getX() > (opponentGoalCenter().getX() - 4))
  {
    needKick = true;
  }
  else
  {
    if (robot_in_line.empty())
    {
      needKick = true;
    }
    else
    {
      needKick = false;
    }
  }

  Vector2d ball_robot_vector = robot_position - ballPosition();
  double dist_ball_robot = ball_robot_vector.norm();
  ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();

  Vector2d ball_point_vector = point_to_pass - ballPosition();
  ball_point_vector = ball_point_vector / ball_point_vector.norm();

  double target_radius_from_ball;
  double scalar_ball_robot = -scalarProduct(ball_robot_vector, ball_point_vector);

  if (scalar_ball_robot < 0)
  {
    follower->avoid_the_ball(true);
    target_radius_from_ball = 0.4;
  }
  else
  {
    follower->avoid_the_ball(false);
    // target_radius_from_ball = 1.0 / ( 4.0*(scalar_ball_robot - 1.4) ) + 0.55;
    target_radius_from_ball = 1.0 / (24.0 * (scalar_ball_robot - 1.04)) + 0.44;

    if (dist_ball_robot < 0.4)
    {
      follower->avoid_opponent(false);
    }
  }
  if (dist_ball_robot > 0.4)
  {
    follower->avoid_opponent(true);
  }

  rhoban_geometry::Point target_position = ballPosition() - ball_point_vector * target_radius_from_ball;
  double target_rotation = detail::vec2angle(ball_point_vector);

  // follower->avoid_the_ball(false);
  follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Degageur::control() const
{
  Control ctrl = follower->control();
  ctrl.charge = true;
  ctrl.kickPower = 1.0;

  if (needKick)
  {
    ctrl.chipKick = false;
    ctrl.kick = true;
  }
  else
  {
    ctrl.chipKick = true;
    ctrl.kick = false;
  }
  return ctrl;
}

void Degageur::declare_point_to_pass(rhoban_geometry::Point point)
{
  point_to_pass = point;
}

void Degageur::declare_robot_to_pass(int robot_id, vision::Team team)
{
  robot_to_pass_id = robot_id;
  robot_to_pass_team = team;
}

Degageur::~Degageur()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Degageur::get_annotations() const
{
  return follower->get_annotations();
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
