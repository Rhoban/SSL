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
#include "lob.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
Lob::Lob(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , follower_(Factory::fixedConsignFollower(ai_data))
  , dist_minimal_to_lob_(ai_data.constants.radius_ball + ai_data.constants.robot_radius)
{
}

void Lob::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);
  Vector2d robot_ball_vector = ballPosition() - robot_position;
  rhoban_geometry::Point target_position = ballPosition();

  follower_->setFollowingPosition(target_position, vector2angle(robot_ball_vector));

  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control Lob::control() const
{
  const rhoban_geometry::Point& robot_position = linearPosition();

  Vector2d robot_ball_vector = robot_position - ballPosition();
  double dist = robot_ball_vector.norm();
  Control ctrl = follower_->control();

  if (dist < dist_minimal_to_lob_)
  {
    ctrl.chipKick = true;
  }
  return ctrl;
}

Lob::~Lob()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Lob::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl
