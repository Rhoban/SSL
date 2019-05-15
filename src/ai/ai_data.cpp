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
#include "ai_data.h"
#include <assert.h>
#include <physic/factory.h>
#include <debug.h>
#include <physic/collision.h>
#include <physic/movement_on_new_frame.h>
#include <json/json.h>
#include <json/reader.h>
#include <fstream>

namespace rhoban_ssl
{
namespace ai
{
RobotPlacement::RobotPlacement() : goal_is_placed(false){};
RobotPlacement::RobotPlacement(std::vector<Position> field_robot_position, Position goalie_position)
  : goal_is_placed(true), field_robot_position(field_robot_position), goalie_position(goalie_position)
{
}
RobotPlacement::RobotPlacement(std::vector<Position> field_robot_position)
  : goal_is_placed(false), field_robot_position(field_robot_position)
{
}

Object::Object(const Object& object) : vision_data(object.vision_data), movement(object.movement->clone())
{
}

Object& Object::operator=(const Object& object)
{
  this->vision_data = vision_data;
  this->movement = object.movement->clone();
  return *this;
}

void Object::setVisionData(const vision::Object& vision_data)
{
  this->vision_data = vision_data;
  this->movement->setSample(this->vision_data.movement);
}
void Object::setMovement(Movement* movement)
{
  if (this->movement)
  {
    assert(movement != static_cast<MovementOnNewFrame*>(this->movement)->getOriginalMovement());
    delete this->movement;
  }
  // We change the frame according referee informatiosns
  this->movement = new MovementOnNewFrame(movement);
}
void Object::changeFrame(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2)
{
  static_cast<MovementOnNewFrame*>(movement)->setFrame(origin, v1, v2);
}

const rhoban_ssl::Movement& Object::getMovement() const
{
  return *movement;
}

Object::~Object()
{
  delete movement;
}

Object::Object() : movement(0)
{
}

bool Object::isPresentInVision() const
{
  return vision_data.isOk();
}

void AiData::update(const vision::VisionData vision_data)
{
  if (vision_data.field.present)
  {
    static_cast<vision::Field&>(field) = vision_data.field;
  };

  for (auto team : { vision::Ally, vision::Opponent })
  {
    for (int k = 0; k < vision::Robots; k++)
    {
      robots[team][k].setVisionData(vision_data.robots.at(team).at(k));
    }
  }
  ball.setVisionData(vision_data.ball);
  computeTableOfCollisionTimes();
}

void AiData::changeFrameForAllObjects(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2)
{
  team_point_of_view.setFrame(origin, v1, v2);
  for (auto team : { vision::Ally, vision::Opponent })
  {
    for (int k = 0; k < vision::Robots; k++)
    {
      robots[team][k].changeFrame(origin, v1, v2);
    }
  }
  ball.changeFrame(origin, v1, v2);
};

void AiData::changeTeamColor(ai::Team team_color)
{
  this->team_color = team_color;
}

AiData::AiData(const std::string& config_path, bool is_in_simulation, ai::Team team_color)
  : time_shift_with_vision(0.0), team_color(team_color), constants(config_path, is_in_simulation), dt(0.0)
{
  dt = constants.period;
  int nb_robots = 0;
  for (auto team : { vision::Ally, vision::Opponent })
  {
    for (int k = 0; k < vision::Robots; k++)
    {
      robots[team][k].setMovement(physic::Factory::robotMovement(*this));
      nb_robots++;
    }
  }
  all_robots = std::vector<std::pair<vision::Team, Robot*> >(nb_robots);
  unsigned int i = 0;
  for (auto team : { vision::Ally, vision::Opponent })
  {
    for (int k = 0; k < vision::Robots; k++)
    {
      all_robots[i] = std::pair<vision::Team, Robot*>(team, &(robots.at(team).at(k)));
      i++;
    }
  }
  ball.setMovement(physic::Factory::ballMovement(*this));
}

Constants::Constants(const std::string& config_path, bool is_in_simulation) : is_in_simulation(is_in_simulation)
{
  load(config_path);
}

void Constants::load(const std::string& config_path)
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

  enable_movement_with_integration = root["movement_prediction"]["enable_integration"].asBool();

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

  radius_ball = root["ball"]["radius_ball"].asDouble();
  assert(radius_ball > 0.0);

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
  default_goalie_id = root["goalie"]["default_id"].asInt();  // penalty rayon for the goalie
  assert(default_goalie_id >= 0);
  assert(default_goalie_id < Constants::NB_OF_ROBOTS_BY_TEAM);

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
}

bool AiData::robotIsInsideTheField(int robot_id) const
{
  const rhoban_ssl::Movement& mov = robots.at(vision::Team::Ally).at(robot_id).getMovement();
  return field.isInside(mov.linearPosition(time));
}

bool AiData::robotIsValid(int robot_id) const
{
  return (robots.at(vision::Team::Ally).at(robot_id).isPresentInVision() and robotIsInsideTheField(robot_id));
}

const AiData::Collision_times_table& AiData::getTableOfCollisionTimes() const
{
  return table_of_collision_times;
}

void AiData::visitAllPairOfRobots(
    std::function<void(vision::Team robot_team_1, Robot& robot_1, vision::Team robot_team_2, Robot& robot_2)> visitor)
{
  for (unsigned int i = 0; i < all_robots.size(); i++)
  {
    for (unsigned int j = i + 1; j < all_robots.size(); j++)
    {
      visitor(all_robots[i].first, *all_robots[i].second, all_robots[j].first, *all_robots[j].second);
    }
  }
}

std::list<std::pair<int, double> > AiData::getCollisions(int robot_id, const Vector2d& linear_velocity) const
{
  std::list<std::pair<int, double> > result;
  const Robot* robot_1 = &(robots.at(vision::Team::Ally).at(robot_id));

  if (not(robot_1->isPresentInVision()))
  {
    return {};
  }

  for (unsigned int i = 0; i < all_robots.size(); i++)
  {
    const Robot* robot_2 = all_robots[i].second;
    if (not(robot_2->isPresentInVision()))
    {
      continue;
    }
    if (robot_1->id() != robot_2->id() or all_robots[i].first != vision::Team::Ally)
    {
      double radius_error = constants.radius_security_for_collision;
      std::pair<bool, double> collision = collisionTime(
          constants.robot_radius, robot_1->getMovement().linearPosition(robot_1->getMovement().lastTime()),
          linear_velocity, constants.robot_radius,
          robot_2->getMovement().linearPosition(robot_2->getMovement().lastTime()),
          robot_2->getMovement().linearVelocity(robot_2->getMovement().lastTime()), radius_error);
      if (collision.first)
      {
        result.push_back(std::pair<int, double>(i, collision.second));
      }
    }
  }
  return result;
}

void AiData::computeTableOfCollisionTimes()
{
  table_of_collision_times.clear();
  for (unsigned int i = 0; i < all_robots.size(); i++)
  {
    for (unsigned int j = i + 1; j < all_robots.size(); j++)
    {
      Robot& robot_1 = *all_robots[i].second;
      Robot& robot_2 = *all_robots[j].second;
      double radius_error = constants.radius_security_for_collision;
      std::pair<bool, double> collision =
          collisionTime(constants.robot_radius, robot_1.getMovement(), constants.robot_radius, robot_2.getMovement(),
                        radius_error, time);
      if (collision.first)
      {
        table_of_collision_times[std::pair<int, int>(i, j)] = collision.second;
      }
    }
  }
}

rhoban_geometry::Point AiData::relative2absolute(double x, double y) const
{
  return rhoban_geometry::Point(field.fieldLength / 2.0 * x, field.fieldWidth / 2.0 * y);
}
rhoban_geometry::Point AiData::relative2absolute(const rhoban_geometry::Point& point) const
{
  return relative2absolute(point.getX(), point.getY());
}

RobotPlacement AiData::defaultAttackingKickoffPlacement() const
{
  //
  //     A.
  // B        C
  //      D
  //   E     F
  //      G
  //
  return RobotPlacement(
      {
          Position(-0.8, .0, 0.0),                                   // A
          Position(relative2absolute(-1.0 / 3.0, 2.0 / 3.0), 0.0),   // B
          Position(relative2absolute(-1.0 / 3.0, -2.0 / 3.0), 0.0),  // C
          Position(relative2absolute(-2.0 / 3.0, 1.0 / 2.0), 0.0),   // D
          Position(relative2absolute(-2.0 / 3.0, -1.0 / 2.0), 0.0),  // E
          Position(relative2absolute(-1.5 / 3.0, 0.0), 0.0),         // F
          Position(relative2absolute(-2.5 / 3.0, 0.0), 0.0),         // G
      },
      Position(relative2absolute(-1.0, 0.0), 0.0  // G
               ));
}

RobotPlacement AiData::defaultDefendingKickoffPlacement() const
{
  //
  //      .
  // B    A    C
  //      F
  //   D     E
  //      G
  //
  return RobotPlacement(
      {
          Position(relative2absolute(-1.0 / 3.0, 0.0), 0.0),         // A
          Position(relative2absolute(-1.0 / 3.0, 2.0 / 3.0), 0.0),   // B
          Position(relative2absolute(-1.0 / 3.0, -2.0 / 3.0), 0.0),  // C
          Position(relative2absolute(-2.0 / 3.0, 1.0 / 2.0), 0.0),   // D
          Position(relative2absolute(-2.0 / 3.0, -1.0 / 2.0), 0.0),  // E
          Position(relative2absolute(-1.5 / 3.0, 0.0), 0.0),         // F
          Position(relative2absolute(-2.5 / 3.0, 0.0), 0.0),         // G
      },
      Position(relative2absolute(-1.0, 0.0), 0.0  // G
               ));
}

std::ostream& operator<<(std::ostream& out, const Object& object)
{
  out << "visible : " << object.isPresentInVision() << ", vision : " << object.vision_data;
  return out;
}

Robot::Robot() : is_goalie(false), infra_red(false)
{
}

}  // namespace ai
}  // namespace rhoban_ssl
