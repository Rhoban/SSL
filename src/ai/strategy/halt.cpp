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

#include "halt.h"
#include <robot_behavior/do_nothing.h>

namespace rhoban_ssl
{
namespace Strategy
{
const std::string Halt::name = "halt";

Halt::Halt(ai::AiData& ai_data) : Strategy(ai_data)
{
}

int Halt::min_robots() const
{
  return 0;
}
int Halt::max_robots() const
{
  return -1;
}
Goalie_need Halt::needs_goalie() const
{
  return Goalie_need::IF_POSSIBLE;
}

void Halt::start(double time)
{
}

void Halt::stop(double time)
{
}

void Halt::assign_behavior_to_robots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (have_to_manage_the_goalie())
  {
    assign_behavior(get_goalie(),
                    std::shared_ptr<Robot_behavior::RobotBehavior>(new Robot_behavior::DoNothing(ai_data)));
  }
  for (int id : get_player_ids())
  {
    assign_behavior(id, std::shared_ptr<Robot_behavior::RobotBehavior>(new Robot_behavior::DoNothing(ai_data)));
  }
}

Halt::~Halt()
{
}

RhobanSSLAnnotation::Annotations Halt::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;

  for (auto it = this->get_player_ids().begin(); it != this->get_player_ids().end(); it++)
  {
    const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linear_position(time());
    // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
    annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
  }
  return annotations;
}

}  // namespace Strategy
}  // namespace rhoban_ssl
