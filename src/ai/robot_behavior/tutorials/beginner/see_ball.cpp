/*
    This file is part of SSL.

    Copyright 2019 SCHMITZ Etienne (hello@etienne-schmitz.com)

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

#include "see_ball.h"
#include <math/vector2d.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace beginner
{
SeeBall::SeeBall(Ai::AiData& ai_data) : RobotBehavior(ai_data), follower_(Factory::fixed_consign_follower(ai_data))
{
}

void SeeBall::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::update_time_and_position(time, robot, ball);
  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.get_movement().linear_position(ai_data.time);

  Vector2d robot_ball = ball_position() - robot_position;
  ContinuousAngle target_angular_position = vector2angle(robot_ball);

  follower_->set_following_position(robot_position, target_angular_position);
  follower_->update(time, robot, ball);
}

Control SeeBall::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

SeeBall::~SeeBall()
{
  delete follower_;
}

RhobanSSLAnnotation::Annotations SeeBall::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->get_annotations());
  return annotations;
}

}  // namespace beginner
}  // namespace Robot_behavior
}  // namespace RhobanSSL
