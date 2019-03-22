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

#ifndef __ROBOT_BEHAVIOR__TUTORIALS__MEDIUM__FOLLOW_ROBOT__H__
#define __ROBOT_BEHAVIOR__TUTORIALS__MEDIUM__FOLLOW_ROBOT__H__

#include "robot_behavior/factory.h"
#include "robot_behavior/robot_behavior.h"

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace medium
{
class FollowRobot : public RobotBehavior
{
private:
  int target_id_;
  ConsignFollower* follower_;
  RhobanSSLAnnotation::Annotations annotations_;
  const double TRACKING_DISTANCE = 0.3;

public:
  FollowRobot(Ai::AiData& ai_data, int target_id = 0);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  virtual Control control() const;

  void setRobotIdToFollow(int target_id);

  int getRobotIdToFollow() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  virtual ~FollowRobot();
};

};  // namespace Medium
};  // namespace Robot_behavior
};  // namespace RhobanSSL

#endif