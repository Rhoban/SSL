/*
    This file is part of SSL.

    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)

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

#include "test_follow_path.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
TestFollowPath::TestFollowPath(std::vector<rhoban_geometry::Point> path)
  : RobotBehavior(), follower_(Factory::fixedConsignFollower()), path_(path), path_index_(0)
{
}

void TestFollowPath::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(time);
  rhoban_geometry::Point next_point = path_.at(path_index_ % path_.size());

  if (robot_position.getDist(next_point) <= REACH_RADIUS)
  {
    path_index_++;
    next_point = path_.at(path_index_ % path_.size());
  }

  Vector2d vect_robot_target = next_point - robot_position;
  ContinuousAngle target_rotation = vector2angle(vect_robot_target);

  follower_->avoidTheBall(false);
  follower_->setFollowingPosition(next_point, target_rotation);
  follower_->update(time, robot, ball);
}

Control TestFollowPath::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

TestFollowPath::~TestFollowPath()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations TestFollowPath::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace tests
}  // namespace robot_behavior
}  // namespace rhoban_ssl
