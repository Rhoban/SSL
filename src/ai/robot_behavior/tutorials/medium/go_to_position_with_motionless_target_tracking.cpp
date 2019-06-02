/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

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
#include "go_to_position_with_motionless_target_tracking.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
GoToPositionWithMotionlessTargetTracking::GoToPositionWithMotionlessTargetTracking()
  : RobotBehavior(), follower_(), circle_follower_()
{
}

void GoToPositionWithMotionlessTargetTracking::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

#if 1
  if (!robot_destination_set_)
  {
    ContinuousAngle angle_destination(M_PI_2);

    circle_follower_.setGoal(angle_destination, ballPosition(), Vector2d(0.0, 0.0));
    robot_destination_set_ = true;
  }
  circle_follower_.update(time, robot, ball);
  annotations_.addCross(
      circle_follower_.robot_control::tracking::RobotControlWithTargetTrackingAndCircleFollowing::linearPosition(time),
      "green");

  annotations_.addArrow(
      circle_follower_.robot_control::tracking::RobotControlWithTargetTrackingAndCircleFollowing::linearPosition(time),
      circle_follower_.robot_control::tracking::RobotControlWithTargetTrackingAndCircleFollowing::linearPosition(time) +
          Vector2d(
              std::cos(
                  circle_follower_
                      .robot_control::tracking::RobotControlWithTargetTrackingAndCircleFollowing::angularPosition(time)
                      .value()),
              std::sin(
                  circle_follower_
                      .robot_control::tracking::RobotControlWithTargetTrackingAndCircleFollowing::angularPosition(time)
                      .value())),
      "green", false);

  // DEBUG(angular_position() - vector2angle(linear_position()-ball_position()));

#else
  if (!robot_destination_set_)
  {
    rhoban_geometry::Point robot_destination;
    robot_destination.x = linear_position().x;
    robot_destination.y = ai_data.field.fieldWidth / 2;
    follower_.setGoal(robot_destination, rhoban_geometry::Point(0, 0), Vector2d(0.0, 0.0));
    robot_destination_set_ = true;
  }

  // DEBUG(follower_.linearPosition(time));
  annotations_.addCross(follower_.linearPosition(time), "green");

  annotations_.addArrow(follower_.linearPosition(time),
                        follower_.linearPosition(time) + Vector2d(std::cos(follower_.angularPosition(time).value()),
                                                                  std::sin(follower_.angularPosition(time).value())),
                        "green", false);

  follower_.update(time, robot, ball);
#endif
}

Control GoToPositionWithMotionlessTargetTracking::control() const
{
  return circle_follower_.control();
}

annotations::Annotations GoToPositionWithMotionlessTargetTracking::getAnnotations() const
{
  annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(circle_follower_.getAnnotations());
  return annotations;
}

}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl
