#pragma once

#include <math/vector2d.h>
#include <execution_manager.h>

namespace rhoban_ssl
{
typedef int Team;
const Team Ally = 0;
const Team Opponent = 1;

using cardinal_position = uint;
static constexpr cardinal_position SW = 0;
static constexpr cardinal_position NW = 1;
static constexpr cardinal_position NE = 2;
static constexpr cardinal_position SE = 3;

namespace ai
{
class UpdateConfigTask : public Task
{
  std::string config_path_;
  double last_time;
  int cpt;

public:
  UpdateConfigTask(std::string confpath);
  bool runTask() override;
};

struct Config
{
  // todo move to GlobalData ?
  static std::string team_name;

  static constexpr int NB_OF_ROBOTS_BY_TEAM = 8;
  static constexpr unsigned int NB_CAMERAS = 4;
  // The Kalman filter doesn't support more than 1 ball per camera
  static constexpr unsigned int MAX_BALLS_DETECTED_PER_CAMERA = 1;

  static std::vector<unsigned int> attackers_;
  static std::vector<unsigned int> defenders_;
  static std::vector<unsigned int> goalies_;

  static bool enable_movement_with_integration;

  static bool we_are_blue;

  static bool is_in_simulation;

  static double period;

  static bool is_in_mixcontrol;
  static bool log_replay;

  static double robot_radius;
  static double ball_radius;
  static double max_wheel_speed;
  static double rear_wheel_angle;
  static double front_wheel_angle;

  static Vector2d waiting_goal_position;
  static uint default_goalie_id;

  // PID for translation
  static double p_translation;
  static double i_translation;
  static double d_translation;

  // PID for orientation
  static double p_orientation;
  static double i_orientation;
  static double d_orientation;

  // PID for translation for goalkeeper
  static double p_translation_goalkeeper;
  static double i_translation_goalkeeper;
  static double d_translation_goalkeeper;

  // PID for orientation for goalkeeper
  static double p_orientation_goalkeeper;
  static double i_orientation_goalkeeper;
  static double d_orientation_goalkeeper;

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

  static double robot_center_to_dribbler_center;
  static double wheel_radius;
  static double wheel_excentricity;
  static double wheel_nb_turns_acceleration_limit;

  static double rules_avoidance_distance;
  static double convergence_coefficient;
  static double coefficient_to_increase_avoidance_convergence;

  static std::vector<std::vector<double>> kick_settings;

  static bool ntpd_enable;

  static void load(const std::string& config_path);
};
}  // namespace ai
}  // namespace rhoban_ssl
