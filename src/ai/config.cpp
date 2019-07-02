#include <json/json.h>
#include <json/reader.h>
#include <fstream>
#include "config.h"
#include "debug.h"

namespace rhoban_ssl
{
namespace ai
{
std::string Config::team_name = "NAMec";

bool Config::enable_movement_with_integration = true;
bool Config::we_are_blue = true;
bool Config::is_in_simulation = true;

int Config::frame_per_second = 60;
double Config::period = 1.0 / 60.0;

double Config::robot_radius = 0.09;
double Config::ball_radius = 0.021375;
Vector2d Config::waiting_goal_position;
uint Config::default_goalie_id;

// PID for translation
double Config::p_translation;
double Config::i_translation;
double Config::d_translation;
// PID for orientation
double Config::p_orientation;
double Config::i_orientation;
double Config::d_orientation;

bool Config::enable_kicking;

double Config::penalty_rayon;
double Config::translation_velocity_limit;
double Config::rotation_velocity_limit;
double Config::translation_acceleration_limit;
double Config::rotation_acceleration_limit;

double Config::time_limit_between_collision;
double Config::security_acceleration_ratio;
double Config::obstacle_avoidance_ratio;

double Config::radius_security_for_collision;
double Config::radius_security_for_avoidance;

double Config::wheel_radius = 0.03;
double Config::wheel_excentricity = 0.08;
double Config::wheel_nb_turns_acceleration_limit = 150.0;
double Config::max_wheel_speed;
double Config::front_wheel_angle;
double Config::rear_wheel_angle;

double Config::rules_avoidance_distance;
double Config::convergence_coefficient;
double Config::coefficient_to_increase_avoidance_convergence;

void Config::load(const std::string& config_path)
{
  DEBUG("We load constants from the configuration file : " << config_path << ".");
  Json::Value root;

  std::ifstream config_doc(config_path, std::ifstream::binary);

  Json::Reader reader;
  bool parsingSuccessful = reader.parse(config_doc, root);
  if (!parsingSuccessful)
  {
    std::cerr << "Failed to parse configuration\n" << reader.getFormattedErrorMessages();
    std::exit(EXIT_FAILURE);
  }

  auto robot_conf = root["robot"];

  frame_per_second = root["time"]["frame_per_second"].asInt();
  assert(frame_per_second > 0);
  period = 1.0 / frame_per_second;

  robot_radius = robot_conf["robot_radius"].asDouble();
  assert(robot_radius > 0.0);

  wheel_radius = robot_conf["wheel_radius"].asDouble();
  assert(wheel_radius > 0.0);

  wheel_excentricity = robot_conf["wheel_excentricity"].asDouble();
  assert(wheel_excentricity > 0.0);

  max_wheel_speed = robot_conf["max_wheel_speed"].asDouble();
  assert(max_wheel_speed > 0.0);

  front_wheel_angle = robot_conf["front_wheel_angle"].asDouble();
  assert(front_wheel_angle > 0.0);

  rear_wheel_angle = robot_conf["rear_wheel_angle"].asDouble();
  assert(rear_wheel_angle > 0.0);

  enable_movement_with_integration = root["movement_prediction"]["enable_integration"].asBool();

  if (is_in_simulation)
  {
    DEBUG("SIMULATION MODE ACTIVATED");
    wheel_nb_turns_acceleration_limit = robot_conf["wheel_nb_turns_acceleration_limit"]["simu"].asDouble();
    rotation_velocity_limit = robot_conf["rotation_velocity_limit"]["simu"].asDouble();
    translation_velocity_limit = robot_conf["translation_velocity_limit"]["simu"].asDouble();
    p_translation = robot_conf["p_translation"]["simu"].asDouble();
    i_translation = robot_conf["i_translation"]["simu"].asDouble();
    d_translation = robot_conf["d_translation"]["simu"].asDouble();
    p_orientation = robot_conf["p_orientation"]["simu"].asDouble();
    i_orientation = robot_conf["i_orientation"]["simu"].asDouble();
    d_orientation = robot_conf["d_orientation"]["simu"].asDouble();
  }
  else
  {
    DEBUG("REAL MODE ACTIVATED");
    wheel_nb_turns_acceleration_limit = robot_conf["wheel_nb_turns_acceleration_limit"]["real"].asDouble();
    rotation_velocity_limit = robot_conf["rotation_velocity_limit"]["real"].asDouble();
    translation_velocity_limit = robot_conf["translation_velocity_limit"]["real"].asDouble();
    p_translation = robot_conf["p_translation"]["real"].asDouble();
    i_translation = robot_conf["i_translation"]["real"].asDouble();
    d_translation = robot_conf["d_translation"]["real"].asDouble();
    p_orientation = robot_conf["p_orientation"]["real"].asDouble();
    i_orientation = robot_conf["i_orientation"]["real"].asDouble();
    d_orientation = robot_conf["d_orientation"]["real"].asDouble();
  }
  assert(wheel_nb_turns_acceleration_limit > 0.0);
  assert(rotation_velocity_limit > 0.0);
  assert(translation_velocity_limit > 0.0);
  assert(p_translation >= 0.0);
  assert(i_translation >= 0.0);
  assert(d_translation >= 0.0);
  assert(p_orientation >= 0.0);
  assert(i_orientation >= 0.0);
  assert(d_orientation >= 0.0);

  translation_acceleration_limit = wheel_nb_turns_acceleration_limit * wheel_radius * 2.0 * M_PI;
  assert(translation_acceleration_limit > 0.0);

  rotation_acceleration_limit = 2.0 * M_PI * (wheel_radius / wheel_excentricity) * wheel_nb_turns_acceleration_limit;
  assert(rotation_acceleration_limit);

  DEBUG("translation_velocity_limit : " << translation_velocity_limit);
  DEBUG("rotation_velocity_limit : " << rotation_velocity_limit);
  DEBUG("translation_acceleration_limit : " << translation_acceleration_limit);
  DEBUG("rotation_acceleration_limit : " << rotation_acceleration_limit);

  rules_avoidance_distance = 0.5 * (1.0 + 10.0 / 100.0);

  ball_radius = root["ball"]["radius_ball"].asDouble();
  assert(ball_radius > 0.0);

  auto nav = root["navigation_with_obstacle"];
  security_acceleration_ratio = nav["security_acceleration_ratio"].asDouble();
  assert(security_acceleration_ratio >= 0.0);

  obstacle_avoidance_ratio =
      nav["obstacle_avoidance_ratio"].asDouble();  // should be lessr than security_acceleration_ratio
  coefficient_to_increase_avoidance_convergence =
      nav["coefficient_to_increase_avoidance_convergence"].asDouble();  // should be lessr than
                                                                        // security_acceleration_ratio
  assert(obstacle_avoidance_ratio >= 0.0);
  assert(obstacle_avoidance_ratio < security_acceleration_ratio);

  radius_security_for_collision = nav["radius_security_for_collision"].asDouble();
  assert(radius_security_for_collision > 0.0);
  radius_security_for_avoidance =
      nav["radius_security_for_avoidance"].asDouble();  // should be greatear than  radius_security_for_collision
  assert(radius_security_for_avoidance > 0.0);
  assert(radius_security_for_collision < radius_security_for_avoidance);

  waiting_goal_position = Vector2d(root["goalie"]["waiting_goal_position"][0].asDouble(),
                                   root["goalie"]["waiting_goal_position"][1].asDouble());
  penalty_rayon = root["goalie"]["penalty_rayon"].asDouble();  // penalty rayon for the goalie
  assert(penalty_rayon > 0.0);
  default_goalie_id = root["goalie"]["default_id"].asUInt();  // penalty rayon for the goalie
  assert(default_goalie_id >= 0);
  assert(default_goalie_id < Config::NB_OF_ROBOTS_BY_TEAM);

  enable_kicking = true;
}
}  // namespace ai
}  // namespace rhoban_ssl
