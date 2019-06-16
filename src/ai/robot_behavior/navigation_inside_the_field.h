/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#pragma once

#include "robot_behavior.h"
#include "navigation_with_obstacle_avoidance.h"
namespace rhoban_ssl
{
namespace robot_behavior
{
/*
 * This is an implementation of the article :
 * "Orbital Obstavle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class NavigationInsideTheField : public ConsignFollower
{
private:
  bool need_to_avoid_the_ball_;
  double saving_ball_radius_avoidance_;

  bool following_position_was_updated_;
  NavigationWithObstacleAvoidance position_follower_;

  Vector2d target_position_;
  ContinuousAngle target_angle_;
  rhoban_geometry::Point deviation_position_;

  rhoban_ssl::annotations::Annotations annotations_;

public:
  NavigationInsideTheField(double time, double dt);

protected:
  void update_control(double time, const data::Robot& robot, const data::Ball& ball);

public:
  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  virtual Control control() const;

  void setTranslationPid(double kp, double ki, double kd);
  void setOrientationPid(double kp, double ki, double kd);

  void setLimits(double translation_velocity_limit, double rotation_velocity_limit,
                 double translation_acceleration_limit, double rotation_acceleration_limit);

  virtual void setFollowingPosition(const rhoban_geometry::Point& position_to_follow, const ContinuousAngle& angle);
  virtual void avoidTheBall(bool value = true);
  virtual void avoidAlly(bool value = true);
  virtual void avoidOpponent(bool value = true);
  virtual void avoidRobot(int id, bool value);

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
  virtual void setRadiusAvoidanceForTheBall(double radius);
};
};  // namespace robot_behavior
};  // namespace rhoban_ssl
