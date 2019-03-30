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
#include <ai_data.h>
#include <math/circular_vector.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
/*
 * This is an implementation of the article :
 * "Orbital Obstacle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class Navigation_with_obstacle_avoidance : public ConsignFollower
{
private:
  bool ignore_the_ball;
  bool ignore_robot_[2 * ai::Constants::NB_OF_ROBOTS_BY_TEAM];  // 2 *  because there is 2 teams.
  double ball_radius_avoidance;
  bool ball_is_the_obstacle;
  PositionFollower position_follower;

  PositionFollower position_follower_avoidance;

  Vector2d target_position;
  ContinuousAngle target_angle;
  CircularVector<rhoban_geometry::Point> obst_vec;
  double min_time_collision;
  int closest_robot;
  int second_closest_robot;
  double second_closest_distance;
  double radius_of_limit_cycle;
  Vector2d limit_cycle_direction;

  struct Obstacle_point_of_view
  {
    rhoban_geometry::Point robot_linear_position;
    Vector2d robot_linear_velocity;
    rhoban_geometry::Point target_linear_position;

    Vector2d limit_cycle_direction;
  };
  Obstacle_point_of_view obstacle_point_of_view;
  double sign_of_avoidance_rotation;
  Control avoidance_control;

  void determine_the_closest_obstacle();
  void compute_the_radius_of_limit_cycle();
  void compute_the_limit_cycle_direction_for_obstacle(const rhoban_geometry::Point& obstacle_linear_position,
                                                      const Vector2d& obstacle_linear_velocity);
  void compute_the_limit_cycle_direction_for_robot();
  void compute_the_limit_cycle_direction_for_ball();
  void compute_the_limit_cycle_direction();
  void convert_cycle_direction_to_linear_and_angular_velocity();

public:
  Navigation_with_obstacle_avoidance(ai::AiData& ai_data, double time, double dt);

protected:
  void update_control(double time, const ai::Robot& robot, const ai::Ball& ball);

public:
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  void set_translation_pid(double kp, double ki, double kd);
  void set_orientation_pid(double kp, double ki, double kd);

  void set_limits(double translation_velocity_limit, double rotation_velocity_limit,
                  double translation_acceleration_limit, double rotation_acceleration_limit);

  virtual void set_following_position(const rhoban_geometry::Point& position_to_follow, const ContinuousAngle& angle);
  virtual void avoid_the_ball(bool value = true);
  virtual void avoid_ally(bool value);
  virtual void avoid_opponent(bool value);
  virtual void avoidRobot(int id, bool value);

  virtual void set_radius_avoidance_for_the_ball(double radius);
  double get_radius_avoidance_for_the_ball();

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
