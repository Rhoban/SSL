#pragma once

#include <math/vector2d.h>

namespace rhoban_ssl
{
typedef int Team;
const Team Ally = 0;
const Team Opponent = 1;

namespace ai
{
struct Config
{
  static constexpr int NB_OF_ROBOTS_BY_TEAM = 16;
  static constexpr unsigned int NB_CAMERAS = 4;
  static constexpr unsigned int MAX_BALLS_DETECTED = 4;

  static bool enable_movement_with_integration;

  static bool we_are_blue;

  static bool is_in_simulation;

  static int frame_per_second;
  static double period;

  static double robot_radius;
  static double radius_ball;
  static Vector2d waiting_goal_position;
  static int default_goalie_id;

  // PID for translation
  static double p_translation;
  static double i_translation;
  static double d_translation;
  // PID for orientation
  static double p_orientation;
  static double i_orientation;
  static double d_orientation;

  static bool enable_kicking;

  static double penalty_rayon;
  static double translation_velocity_limit;
  static double rotation_velocity_limit;
  static double translation_acceleration_limit;
  static double rotation_acceleration_limit;

  static double time_limit_between_collision;
  static double security_acceleration_ratio;
  static double obstacle_avoidance_ratio;

  static double radius_security_for_collision;
  static double radius_security_for_avoidance;

  static double wheel_radius;
  static double wheel_excentricity;
  static double wheel_nb_turns_acceleration_limit;

  static double rules_avoidance_distance;
  static double convergence_coefficient;
  static double coefficient_to_increase_avoidance_convergence;

  static void load(const std::string& config_path);
};
}
}
