/*
    This file is part of SSL.

    Copyright 2018 Schmitz Etienne (hello@etienne-schmitz.com)

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
#include "defensor.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
Begginer_defensor::Begginer_defensor(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower(Factory::fixedConsignFollower(ai_data))
{
}

void Begginer_defensor::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);

  Vector2d ball_goal_vector = allyGoalCenter() - ballPosition();
  ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();

  // Put the robot at 0.5 meters on the ball on the vector opponent_goal and ball.
  rhoban_geometry::Point target_position = ballPosition() + ball_goal_vector * 0.5;
  double target_rotation = detail::vec2angle(ball_goal_vector);

  follower->setFollowingPosition(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Begginer_defensor::control() const
{
  Control ctrl = follower->control();
  return ctrl;
}

Begginer_defensor::~Begginer_defensor()
{
  delete follower;
}

rhoban_ssl::annotations::Annotations Begginer_defensor::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  annotations.addAnnotations(follower->getAnnotations());
  return annotations;
}

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
