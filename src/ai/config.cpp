#include <json/json.h>
#include <json/reader.h>
#include <fstream>
#include "config.h"
#include "debug.h"
#include <data.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace rhoban_ssl
{
namespace ai
{
std::string Config::team_name = "nAMeC";

std::vector<unsigned int> Config::attackers_;
std::vector<unsigned int> Config::defenders_;
std::vector<unsigned int> Config::goalies_;

bool Config::enable_movement_with_integration = true;
bool Config::we_are_blue = true;
bool Config::is_in_simulation = true;
bool Config::is_in_mixcontrol = false;
double Config::period = 0.01;
bool Config::ntpd_enable = false;

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

// PID for translation
double Config::p_translation_goalkeeper;
double Config::i_translation_goalkeeper;
double Config::d_translation_goalkeeper;

double Config::p_orientation_goalkeeper;
double Config::i_orientation_goalkeeper;
double Config::d_orientation_goalkeeper;

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

double Config::robot_center_to_dribbler_center = 0.055;
double Config::wheel_radius = 0.03;
double Config::wheel_excentricity = 0.08;
double Config::wheel_nb_turns_acceleration_limit = 150.0;
double Config::max_wheel_speed;
double Config::front_wheel_angle;
double Config::rear_wheel_angle;

double Config::rules_avoidance_distance;
double Config::convergence_coefficient;
double Config::coefficient_to_increase_avoidance_convergence;

std::vector<std::vector<double>> Config::kick_settings;

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

  period = root["time"]["period"].asDouble();
  assert(period > 0);

  ntpd_enable = root["time"]["ntpd_enable"].asBool();

  robot_center_to_dribbler_center = robot_conf["robot_center_to_dribbler_center"].asDouble();
  assert(robot_center_to_dribbler_center > 0.0);

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

    // PID for translation
    p_translation_goalkeeper = robot_conf["p_translation_goalkeeper"]["simu"].asDouble();
    i_translation_goalkeeper = robot_conf["i_translation_goalkeeper"]["simu"].asDouble();
    d_translation_goalkeeper = robot_conf["d_translation_goalkeeper"]["simu"].asDouble();

    p_orientation_goalkeeper = robot_conf["p_orientation_goalkeeper"]["simu"].asDouble();
    i_orientation_goalkeeper = robot_conf["i_orientation_goalkeeper"]["simu"].asDouble();
    d_orientation_goalkeeper = robot_conf["d_orientation_goalkeeper"]["simu"].asDouble();
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

    p_translation_goalkeeper = robot_conf["p_translation_goalkeeper"]["real"].asDouble();
    i_translation_goalkeeper = robot_conf["i_translation_goalkeeper"]["real"].asDouble();
    d_translation_goalkeeper = robot_conf["d_translation_goalkeeper"]["real"].asDouble();

    p_orientation_goalkeeper = robot_conf["p_orientation_goalkeeper"]["real"].asDouble();
    i_orientation_goalkeeper = robot_conf["i_orientation_goalkeeper"]["real"].asDouble();
    d_orientation_goalkeeper = robot_conf["d_orientation_goalkeeper"]["real"].asDouble();
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

  assert(p_orientation_goalkeeper >= 0.0);
  assert(i_orientation_goalkeeper >= 0.0);
  assert(d_orientation_goalkeeper >= 0.0);

  assert(p_translation_goalkeeper >= 0.0);
  assert(i_translation_goalkeeper >= 0.0);
  assert(d_translation_goalkeeper >= 0.0);

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
  Data::get()->referee.teams_info->goalkeeper_number = default_goalie_id;
  assert(default_goalie_id >= 0);
  assert(default_goalie_id < Config::NB_OF_ROBOTS_BY_TEAM);

  // load kick settings:
  int nb_robots_in_team = root["nb_robots"].asInt();
  assert(nb_robots_in_team >= 0);
  int nb_points = root["nb_points"].asInt();
  assert(nb_points > 0);
  for (int i = 0; i < nb_robots_in_team; i++)
  {
    kick_settings.push_back(std::vector<double>());
    for (int j = 0; j < nb_points; j++)
    {
      double distance = root["robots"][i]["kick_curve"][j].asDouble();
      assert(distance >= 0.0);
      kick_settings.back().push_back(distance);
    }
  }

  enable_kicking = true;

  DEBUG("test");

  for (unsigned int i = 0; i < 16; i++)
  {
    if (!root["roles"][std::to_string(i)].isString())
    {
      continue;
    }
    std::string robot_role = root["roles"][std::to_string(i)].asString();
    if (robot_role == "attacker")
    {
      attackers_.push_back(i);
    }
    else if (robot_role == "defender")
    {
      defenders_.push_back(i);
    }
    else if (robot_role == "goalie")
    {
      goalies_.push_back(i);
    }
    else
    {
      std::cerr << "Doesn't know this role" << std::endl;
    }
  }

  for (auto it = root["can_be_goalie"].begin(); it != root["can_be_goalie"].end(); it++)
  {
    unsigned int id = (*it).asUInt();
    goalies_.push_back(id);
  }
}

double get_time_for_file(std::string file)
{
  struct stat buf;
  if (stat(file.c_str(), &buf) == -1)
    return 0;
  // std::cout << "get time for file " << buf.st_mtim.tv_sec << std::endl;
  return buf.st_mtim.tv_sec;
}

UpdateConfigTask::UpdateConfigTask(std::string confpath) : config_path_(confpath), cpt(0)
{
  ai::Config::load((config_path_));
  last_time = get_time_for_file(config_path_);
}

bool UpdateConfigTask::runTask()
{
  cpt += 1;
  if (cpt > 100)
  {
    cpt = 0;
    double t = get_time_for_file(config_path_);
    if (t > last_time)
    {
      std::cout << "reload config file" << std::endl;
      ai::Config::load((config_path_));
      last_time = t;
    }
  }
  return true;
}

}  // namespace ai
}  // namespace rhoban_ssl
