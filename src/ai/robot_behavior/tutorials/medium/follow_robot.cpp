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

#include "follow_robot.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
/*
 * With this behavior, the robot will go to its target until it reach a defined distance.
 * In short it follow its target without collide it.
 */
FollowRobot::FollowRobot(ai::AiData& ai_data, int target_id)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data))

{
  target_id_ = target_id;
}

void FollowRobot::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point follow_position = robot.getMovement().linearPosition(ai_data_.time);
  ContinuousAngle follow_rotation = robot.getMovement().angularPosition(ai_data_.time);

  // Condition to check if the target robot is not the robot itself.
  // A robot which try to follow itself will do nothing.
  if (target_id_ != robot.id())
  {
    rhoban_geometry::Point target_position = getRobot(target_id_).getMovement().linearPosition(ai_data_.time);
    rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(ai_data_.time);

    Vector2d vect_robot_target = target_position - robot_position;
    follow_rotation = vector2angle(vect_robot_target);

    // Condition to move only if we are away the target (dist > tracking distance radius),
    // that also avoid division by zero.
    if (robot_position.getDist(target_position) > TRACKING_DISTANCE)
    {
      vect_robot_target *= (1 - TRACKING_DISTANCE / vect_robot_target.norm());  // reduce vector
      follow_position = robot_position + vector2point(vect_robot_target);       // transfom vector extremity to point.
    }
  }

  // Disabled avoidance with the target to correclty follow it:
  follower_->avoidRobot(target_id_, false);
  follower_->setFollowingPosition(follow_position, follow_rotation);

  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

void FollowRobot::setRobotIdToFollow(int target_id)
{
  target_id_ = target_id;
}

int FollowRobot::getRobotIdToFollow() const
{
  return target_id_;
}

Control FollowRobot::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

FollowRobot::~FollowRobot()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations FollowRobot::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl