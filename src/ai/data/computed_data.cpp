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
#include "computed_data.h"
#include <config.h>
#include "physic/collision.h"

namespace rhoban_ssl
{
namespace data
{
CollisionComputing::CollisionComputing()
{
}

std::list<std::pair<int, double> > CollisionComputing::getCollisions(int robot_id, const Vector2d& linear_velocity)
{
  std::list<std::pair<int, double> > result;
  const Robot* robot_1 = &(Data::get()->robots[Ally][robot_id]);

  if (not(robot_1->isActive()))
  {
    return {};
  }

  for (unsigned int i = 0; i < Data::get()->all_robots.size(); i++)
  {
    const Robot* robot_2 = Data::get()->all_robots[i].second;
    if (not(robot_2->isActive()))
    {
      continue;
    }
    if (robot_1->id != robot_2->id or Data::get()->all_robots[i].first != Ally)
    {
      double radius_error = ai::Config::radius_security_for_collision;
      std::pair<bool, double> collision = collisionTime(
          ai::Config::robot_radius, robot_1->movement->linearPosition(robot_1->movement->lastTime()), linear_velocity,
          ai::Config::robot_radius, robot_2->movement->linearPosition(robot_2->movement->lastTime()),
          robot_2->movement->linearVelocity(robot_2->movement->lastTime()), radius_error);
      if (collision.first)
      {
        result.push_back(std::pair<int, double>(i, collision.second));
      }
    }
  }
  return result;
}

void CollisionComputing::computeTableOfCollisionTimes()
{
  Data::get()->ai_data.table_of_collision_times_.clear();
  for (unsigned int i = 0; i < Data::get()->all_robots.size(); i++)
  {
    for (unsigned int j = i + 1; j < Data::get()->all_robots.size(); j++)
    {
      Robot& robot_1 = *Data::get()->all_robots[i].second;
      Robot& robot_2 = *Data::get()->all_robots[j].second;
      if ((robot_1.isActive()) && (robot_2.isActive()))
      {
        double radius_error = ai::Config::radius_security_for_collision;
        std::pair<bool, double> collision =
            collisionTime(ai::Config::robot_radius, *robot_1.movement, ai::Config::robot_radius, *robot_2.movement,
                          radius_error, Data::get()->ai_data.time);
        if (collision.first)
        {
          Data::get()->ai_data.table_of_collision_times_[std::pair<int, int>(i, j)] = collision.second;
        }
      }
    }
  }
}

bool CollisionComputing::runTask()
{
  computeTableOfCollisionTimes();
  return true;
}

}  // namespace data

}  // namespace rhoban_ssl
