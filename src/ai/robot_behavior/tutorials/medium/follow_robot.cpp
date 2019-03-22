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

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace medium
{
/*
 * With this behavior, the robot will go to its target until it reach a defined distance.
 * In short it follow its target without collide it.
 */
FollowRobot::FollowRobot(Ai::AiData& ai_data, int target_id)
  : RobotBehavior(ai_data), follower_(Factory::fixed_consign_follower(ai_data))

{
  target_id_ = target_id;
}

void FollowRobot::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point follow_position = robot.get_movement().linear_position(ai_data.time);
  ContinuousAngle follow_rotation = robot.get_movement().angular_position(ai_data.time);

  // Condition to check if the target robot is not the robot itself.
  // A robot which try to follow itself will do nothing.
  if (target_id_ != robot.id())
  {
    rhoban_geometry::Point target_position = get_robot(target_id_).get_movement().linear_position(ai_data.time);
    rhoban_geometry::Point robot_position = robot.get_movement().linear_position(ai_data.time);

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

  follower_->set_following_position(follow_position, follow_rotation);

  follower_->avoid_the_ball(false);
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

RhobanSSLAnnotation::Annotations FollowRobot::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->get_annotations());
  return annotations;
}

}  // namespace Medium
}  // namespace Robot_behavior
}  // namespace RhobanSSL