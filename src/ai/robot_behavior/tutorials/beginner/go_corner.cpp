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
#include <math/vector2d.h>
#include "go_corner.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace beginner
{
// Use opponent_corner_left() for the left corner.
GoCorner::GoCorner(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data)), target_corner_(opponentCornerLeft())
{
}

void GoCorner::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  annotations_.clear();

  const rhoban_geometry::Point& future_position = target_corner_;
  ContinuousAngle angle(0.0);

  follower_->setFollowingPosition(future_position, angle);
  follower_->update(time, robot, ball);
}

Control GoCorner::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

GoCorner::~GoCorner()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations GoCorner::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace beginner
}  // namespace Robot_behavior
}  // namespace rhoban_ssl
