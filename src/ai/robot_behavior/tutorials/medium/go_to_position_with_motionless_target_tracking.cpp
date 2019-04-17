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
GoToPositionWithMotionlessTargetTracking::GoToPositionWithMotionlessTargetTracking(RhobanSSL::Ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(ai_data)
{
}

void GoToPositionWithMotionlessTargetTracking::update(double time, const RhobanSSL::Ai::Robot& robot,
                                                      const RhobanSSL::Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);

  annotations_.clear();

  if (!robot_destination_set_)
  {
    rhoban_geometry::Point robot_destination;
    robot_destination.x = linear_position().x;
    robot_destination.y = ai_data.field.fieldWidth/2;
    follower_.setGoal(robot_destination, rhoban_geometry::Point(0, 0), Vector2d(0.0, 0.0));
    robot_destination_set_ = true;
  }

  //DEBUG(follower_.linearPosition(time));
  annotations_.addCross(follower_.linearPosition(time), "green");

  annotations_.addArrow(follower_.linearPosition(time),
                       follower_.linearPosition(time)+
                            Vector2d(std::cos(follower_.angularPosition(time).value()), std::sin(follower_.angularPosition(time).value())),
                       "green", false);

  follower_.update(time, robot, ball);
}

Control GoToPositionWithMotionlessTargetTracking::control() const
{
  return follower_.control();
}

RhobanSSLAnnotation::Annotations GoToPositionWithMotionlessTargetTracking::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations( annotations_);
  annotations.addAnnotations( follower_.get_annotations() );
  return annotations;
}

}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl
