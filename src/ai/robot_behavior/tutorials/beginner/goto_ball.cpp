/*
    This file is part of SSL.

    Copyright 2019 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "goto_ball.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
namespace Beginner
{
Goto_ball::Goto_ball(ai::AiData& ai_data) : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data))
{
}

void Goto_ball::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);

  annotations.clear();

  rhoban_geometry::Point robot_position = ballPosition();
  ContinuousAngle angle = 0.0;

  follower->set_following_position(robot_position, angle);
  follower->avoid_the_ball(false);
  follower->update(time, robot, ball);
}

Control Goto_ball::control() const
{
  Control ctrl = follower->control();
  return ctrl;
}

Goto_ball::~Goto_ball()
{
  delete follower;
}

rhoban_ssl::annotations::Annotations Goto_ball::get_annotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  annotations.addAnnotations(follower->get_annotations());
  return annotations;
}

}  // namespace Beginner
}  // namespace Robot_behavior
}  // namespace rhoban_ssl
