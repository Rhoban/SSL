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

#include "caterpillar.h"
#include <robot_behavior/do_nothing.h>

namespace rhoban_ssl
{
namespace strategy
{
const std::string Caterpillar::name = "LA CHENILLE !";

Caterpillar::Caterpillar() : Strategy()
{
  for (uint i = 1; i < getPlayerIds().size(); i++)
  {
    behaviors_.push_back(
        std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()));
  }
}

int Caterpillar::minRobots() const
{
  return 1;
}
int Caterpillar::maxRobots() const
{
  return 1;
}
GoalieNeed Caterpillar::needsGoalie() const
{
  return GoalieNeed::IF_POSSIBLE;
}

void Caterpillar::start(double time)
{
}

void Caterpillar::stop(double time)
{
}

void Caterpillar::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  DEBUG("#################");
  for (uint i = 1; i < getPlayerIds().size(); i++)
  {
    assign_behavior(i, behaviors_.at(i - 1));
  }
}

Caterpillar::~Caterpillar()
{
}

rhoban_ssl::annotations::Annotations Caterpillar::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;

  for (auto it = this->getPlayerIds().begin(); it != this->getPlayerIds().end(); it++)
  {
    const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linearPosition(time());
    // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
    annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
  }
  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl
