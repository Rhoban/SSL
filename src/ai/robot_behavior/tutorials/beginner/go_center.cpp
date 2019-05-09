/*
    This file is part of SSL.

    Copyright 2019 Keryell-Even Mael (keryelleven.mael@gmail.com)

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
#include "go_center.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace beginner
{

GoCenter::GoCenter(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data))
{
}

void GoCenter::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // We start by updating time and position from the abstract class robot_behavior.
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);

  const rhoban_geometry::Point& target_position = centerMark();
  double target_rotation = 0.0;

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->update(time, robot, ball);
}


Control GoCenter::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

GoCenter::~GoCenter()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations GoCenter::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace beginner
}  // namespace robot_behavior
}  // namespace rhoban_ssl
