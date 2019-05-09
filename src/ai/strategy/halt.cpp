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
namespace strategy
{
const std::string Halt::name = "halt";

Halt::Halt(ai::AiData& ai_data) : Strategy(ai_data)
{
}

int Halt::minRobots() const
{
  return 0;
}
int Halt::maxRobots() const
{
  return -1;
}
GoalieNeed Halt::needsGoalie() const
{
  return GoalieNeed::IF_POSSIBLE;
}

void Halt::start(double time)
{
}

void Halt::stop(double time)
{
}

void Halt::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (haveToManageTheGoalie())
  {
    assign_behavior(getGoalie(),
                    std::shared_ptr<robot_behavior::RobotBehavior>(new robot_behavior::DoNothing(ai_data_)));
  }
  for (int id : getPlayerIds())
  {
    assign_behavior(id, std::shared_ptr<robot_behavior::RobotBehavior>(new robot_behavior::DoNothing(ai_data_)));
  }
}

Halt::~Halt()
{
}

rhoban_ssl::annotations::Annotations Halt::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;

  for (auto it = this->getPlayerIds().begin(); it != this->getPlayerIds().end(); it++)
  {
    if (getRobot(*it).vision_data.present)
    {
      const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linearPosition(time());
      // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
      annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "whit"
                                                                                                                 "e");
    }
  }
  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl
