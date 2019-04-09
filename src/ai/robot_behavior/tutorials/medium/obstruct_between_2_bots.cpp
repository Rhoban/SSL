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

#include "obstruct_between_2_bots.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace medium
{
ObstructBetween2Bots::ObstructBetween2Bots(ai::AiData& ai_data, int target_id_1, int target_id_2)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data))

{
  target_id_1_ = target_id_1;
  target_id_2_ = target_id_2;
}

void ObstructBetween2Bots::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  rhoban_geometry::Point robot_position = robot.getMovement().linearPosition(ai_data_.time);
  ContinuousAngle robot_rotation = robot.getMovement().angularPosition(ai_data_.time);

  // Condition to check if one of the targets robots is not the robot itself.
  // A robot which try to obstruct itself will do nothing.
  if (target_id_1_ != robot.id() && target_id_2_ != robot.id())
  {
    rhoban_geometry::Point target_position_1 = getRobot(target_id_1_).getMovement().linearPosition(ai_data_.time);
    rhoban_geometry::Point target_position_2 = getRobot(target_id_2_).getMovement().linearPosition(ai_data_.time);

    Vector2d vect_targets = target_position_1 - target_position_2;

    robot_rotation = vector2angle(vect_targets);  // robot look the first target

    vect_targets /= 2;  // find middle
    robot_position = target_position_2 + vector2point(vect_targets);
  }

  follower_->setFollowingPosition(robot_position, robot_rotation);

  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

void ObstructBetween2Bots::setRobotIDsToObstruct(int target_id_1, int target_id_2)
{
  target_id_1_ = target_id_1;
  target_id_2_ = target_id_2;
}

std::tuple<int, int> ObstructBetween2Bots::getRobotIDsToObstruct() const
{
  return std::make_tuple(target_id_1_, target_id_2_);
}

Control ObstructBetween2Bots::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

ObstructBetween2Bots::~ObstructBetween2Bots()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations ObstructBetween2Bots::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace medium
}  // namespace robot_behavior
}  // namespace rhoban_ssl