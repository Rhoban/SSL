/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#pragma once

#include "robot_behavior.h"
#include "factory.h"

namespace rhoban_ssl
{
namespace Robot_behavior
{
class Degageur : public RobotBehavior
{
private:
  rhoban_geometry::Point point_to_pass;
  int robot_to_pass_id;
  Vision::Team robot_to_pass_team;
  bool needKick;

  ConsignFollower* follower;

public:
  Degageur(Ai::AiData& ai_data);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);
  void declare_point_to_pass(rhoban_geometry::Point point);
  void declare_robot_to_pass(int robot_id, Vision::Team team = Vision::Team::Ally);

  virtual Control control() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  virtual ~Degageur();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
