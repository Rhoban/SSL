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
#include "position_follower.h"
#include <math/circular_vector.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
/*
 * This is an implementation of the article :
 * "Orbital Obstacle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class NavigationWithObstacleAvoidance : public ConsignFollower
{
private:
  bool avoided_;
  const double AVOID_MOMENT = 0.35;                // time in second before collision, when robot begin avoidance.
  const double INFINITE_DODGING_PREVENTION = 0.5;  // empirical

  bool ignore_the_ball_;
  bool ignore_robot_[2 * ai::Config::NB_OF_ROBOTS_BY_TEAM];  // 2 *  because there is 2 teams.
  double ball_radius_avoidance_;
  bool ball_is_the_obstacle_;
  PositionFollower position_follower_;

  PositionFollower position_follower_avoidance_;

  Vector2d target_position_;
  ContinuousAngle target_angle_;
  CircularVector<rhoban_geometry::Point> obst_vec_;
  double min_time_collision_;
  int closest_robot_;
  int second_closest_robot_;
  double second_closest_distance_;
  double radius_of_limit_cycle_;
  Vector2d limit_cycle_direction_;

  struct ObstaclePointOfView
  {
    rhoban_geometry::Point robot_linear_position;
    Vector2d robot_linear_velocity;
    rhoban_geometry::Point target_linear_position;

    Vector2d limit_cycle_direction;
  };
  ObstaclePointOfView obstacle_point_of_view_;
  double sign_of_avoidance_rotation_;
  Control avoidance_control_;

  void determineTheClosestObstacle();
  void computeTheRadiusOfLimitCycle();
  void computeTheLimitCycleDirectionForObstacle(const rhoban_geometry::Point& obstacle_linear_position,
                                                const Vector2d& obstacle_linear_velocity);
  void computeTheLimitCycleDirectionForRobot();
  void computeTheLimitCycleDirectionForBall();
  void computeTheLimitCycleDirection();
  void convertCycleDirectionToLinearAndAngularVelocity();

public:
  NavigationWithObstacleAvoidance(double time, double dt);

protected:
  void updateControl(double time, const data::Robot& robot, const data::Ball& ball);

public:
  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  virtual Control control() const;

  void setTranslationPid(double kp, double ki, double kd);
  void setOrientationPid(double kp, double ki, double kd);

  void setLimits(double translation_velocity_limit, double rotation_velocity_limit,
                 double translation_acceleration_limit, double rotation_acceleration_limit);

  virtual void setFollowingPosition(const rhoban_geometry::Point& position_to_follow, const ContinuousAngle& angle);
  virtual void avoidTheBall(bool value = true);
  virtual void avoidAlly(bool value);
  virtual void avoidOpponent(bool value);
  virtual void avoidRobot(int id, bool value);

  virtual void setRadiusAvoidanceForTheBall(double radius);
  double getRadiusAvoidanceForTheBall();

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
