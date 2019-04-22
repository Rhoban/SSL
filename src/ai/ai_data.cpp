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

#include "config.h"

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
  this->vision_data = object.vision_data;
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

Object::Object() : movement(nullptr)
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
    for (int k = 0; k < ai::Config::NB_OF_ROBOTS_BY_TEAM; k++)
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
    for (int k = 0; k < ai::Config::NB_OF_ROBOTS_BY_TEAM; k++)
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
  : time_shift_with_vision(0.0), dt(0.0), team_color(team_color)
{
  Config::load(config_path);
  Config::is_in_simulation = is_in_simulation;
  dt = Config::period;
  int nb_robots = 0;
  for (auto team : { vision::Ally, vision::Opponent })
  {
    for (int k = 0; k < ai::Config::NB_OF_ROBOTS_BY_TEAM; k++)
    {
      robots[team][k].setMovement(physic::Factory::robotMovement(*this));
      nb_robots++;
    }
  }
  all_robots = std::vector<std::pair<vision::Team, Robot*> >(nb_robots);
  unsigned int i = 0;
  for (auto team : { vision::Ally, vision::Opponent })
  {
    for (int k = 0; k < ai::Config::NB_OF_ROBOTS_BY_TEAM; k++)
    {
      all_robots[i] = std::pair<vision::Team, Robot*>(team, &(robots.at(team).at(k)));
      i++;
    }
  }
  ball.setMovement(physic::Factory::ballMovement(*this));
}


bool AiData::robotIsInsideTheField(int robot_id) const
{
  const rhoban_ssl::Movement& mov = robots.at(vision::Ally).at(robot_id).getMovement();
  return field.isInside(mov.linearPosition(time));
}

bool AiData::robotIsValid(int robot_id) const
{
  return (robots.at(vision::Ally).at(robot_id).isPresentInVision() and robotIsInsideTheField(robot_id));
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
  const Robot* robot_1 = &(robots.at(vision::Ally).at(robot_id));

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
    if (robot_1->id() != robot_2->id() or all_robots[i].first != vision::Ally)
    {
      double radius_error = Config::radius_security_for_collision;
      std::pair<bool, double> collision =
          collisionTime(Config::robot_radius, robot_1->getMovement().linearPosition(robot_1->getMovement().lastTime()),
                        linear_velocity, Config::robot_radius,
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
      double radius_error = Config::radius_security_for_collision;
      std::pair<bool, double> collision = collisionTime(
          Config::robot_radius, robot_1.getMovement(), Config::robot_radius, robot_2.getMovement(), radius_error, time);
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
