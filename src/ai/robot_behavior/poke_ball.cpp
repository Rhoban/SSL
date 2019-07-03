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

#include "poke_ball.h"
#include <math/vector2d.h>
#include <debug.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
PokeBall::PokeBall() : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
}

void PokeBall::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(Data::get()->ai_data.time);

  Vector2d direction = poke_direction_ - robot_position;
  ContinuousAngle target_rotation = vector2angle(direction);

  ContinuousAngle robot_rotation = robot.getMovement().angularPosition(Data::get()->ai_data.time);
  ContinuousAngle diff_angle = target_rotation - robot_rotation;
  
  // DEBUG("deggg " << diff_angle.angle().DEG;)
  if(diff_angle.abs() < 20){
    ready_to_kick_ = true;
  }
  else{
    ready_to_kick_ = false;
  }
  
  follower_->setFollowingPosition(poke_direction_, target_rotation);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control PokeBall::control() const
{
  Control ctrl = follower_->control();  
  if (ready_to_kick_)
  {
    ctrl.kick_power = kick_power_;
    ctrl.charge = true;
    ctrl.kick = true;
  }
  else
  {
    ctrl.charge = false;
    ctrl.kick = false;
  }
    
  return ctrl;
}

void PokeBall::setPokeDirection(rhoban_geometry::Point poke_direction){
  poke_direction_ = poke_direction;
}

void PokeBall::setKickPower(double kick_power){
  if (kick_power > 1)
  {
    kick_power_ = 1;
  }
  else if (kick_power < 0)
  {
    kick_power_ = 0;
  }
  else
  {
    kick_power_ = kick_power;
  }
  
}

PokeBall::~PokeBall()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations PokeBall::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}
}  // namespace robot_behavior
}  // namespace rhoban_ssl
