/*
    This file is part of SSL.

    Copyright 2019 SCHMITZ Etienne (hello@etienne-schmitz.com)

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

#include "tutorial.h"
#include <strategy/from_robot_behavior.h>

#include <robot_behavior/tutorials/beginner/see_ball.h>

namespace rhoban_ssl
{
namespace manager
{
Tutorial::Tutorial(ai::AiData& ai_data) : Manager(ai_data)
{
  registerStrategy("Beginner - See ball", std::shared_ptr<strategy::Strategy>(new strategy::FromRobotBehavior(
                                              ai_data,
                                              [&](double time, double dt) {
                                                robot_behavior::beginner::SeeBall* see_ball =
                                                    new robot_behavior::beginner::SeeBall(ai_data);
                                                return std::shared_ptr<robot_behavior::RobotBehavior>(see_ball);
                                              },
                                              false  // we don't want to define a goal here !
                                              )));
}

void Tutorial::update(double time)
{
  updateCurrentStrategies(time);
}

Tutorial::~Tutorial()
{
}
}  // namespace manager

}  // namespace rhoban_ssl
