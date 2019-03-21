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
#ifndef __POSITION_FOLLOWER_WITH_TARGET_TRACKING__H
#define __POSITION_FOLLOWER_WITH_TARGET_TRACKING__H

#include "robot_behavior.h"
#include <control/tracking/robot_control_with_target_tracking_and_position_following.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
class PositionFollowerWithTargetTracking
  : public robot_control::tracking::RobotControlWithTargetTrackingAndPositionFollowing,
    public RhobanSSL::Robot_behavior::RobotBehavior
{
public:
  PositionFollowerWithTargetTracking(RhobanSSL::Ai::AiData& ai_data);

  // RobotBehavior interface
public:
  void update(double time, const RhobanSSL::Ai::Robot& robot, const RhobanSSL::Ai::Ball& ball);
  Control control() const;
  RhobanSSLAnnotation::Annotations get_annotations() const;

private:
  bool is_time_init_ = false;
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl

#endif  // POSITION_FOLLOWER_WITH_TARGET_TRACKING_H
