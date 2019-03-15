/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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
#include "go_corner.h"
#include <math/vector2d.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
namespace Beginner
{

Go_corner::Go_corner(Ai::AiData& ai_data) : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data))
{
}

void Go_corner::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);
  annotations.clear();

  // Set the robot_position to the right corner. (Use opponent_corner_left() for the left corner).
  const rhoban_geometry::Point& future_position = opponent_corner_right();

  ContinuousAngle angle(0.0);

  follower->set_following_position(future_position, angle);
  follower->update(time, robot, ball);
}

Control Go_corner::control() const
{
  Control ctrl = follower->control();
  return ctrl;
}

Go_corner::~Go_corner()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Go_corner::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  annotations.addAnnotations(this->annotations);
  annotations.addAnnotations(follower->get_annotations());
  return annotations;
}

}  // namespace Beginner
}  // namespace Robot_behavior
}  // namespace RhobanSSL
