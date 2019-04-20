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

#include "example.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
#define PERIOD 10.0

Example::Example(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data)), period_(PERIOD), last_time_(0)
{
}

void Example::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(ai_data_.time);

  // On ajoute un text à la position du robot.
  // annotations.addText("Exemple :)", robot_position, "blue");

  Vector2d direction = ballPosition() - robot_position;
  ContinuousAngle target_rotation = vector2angle(direction);

  if (time - last_time_ > period_)
  {
    rhoban_geometry::Point target;
    if (cpt_ % 2 == 0)
    {
      target = centerAllyField();
    }
    else
    {
      target = centerOpponentField();
    }
    follower_->setFollowingPosition(target, target_rotation);
    cpt_ = (cpt_ + 1) % 2;
    last_time_ = time;
  }

  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control Example::control() const
{
  Control ctrl = follower_->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

Example::~Example()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Example::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
