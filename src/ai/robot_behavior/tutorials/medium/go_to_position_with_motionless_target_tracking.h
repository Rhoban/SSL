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
#ifndef __GO_TO_POSITION__MOTIONLESS_TARGET_TRACKING__H__
#define __GO_TO_POSITION__MOTIONLESS_TARGET_TRACKING__H__

#include <robot_behavior/robot_behavior.h>
#include <robot_behavior/position_follower_with_target_tracking.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
/**
 * @brief The GoToPositionWithMotionlessTargetTracking class
 * @todo add documentation for this file
 */
class GoToPositionWithMotionlessTargetTracking : public RhobanSSL::Robot_behavior::RobotBehavior
{
public:
  GoToPositionWithMotionlessTargetTracking(RhobanSSL::Ai::AiData& ai_data);

  // RobotBehavior interface
public:
  void update(double time, const RhobanSSL::Ai::Robot& robot, const RhobanSSL::Ai::Ball& ball);
  Control control() const;
  RhobanSSLAnnotation::Annotations get_annotations() const;

private:
  PositionFollowerWithTargetTracking follower_;
  bool robot_destination_set_ = false;
  RhobanSSLAnnotation::Annotations annotations_;
};

}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl

#endif
