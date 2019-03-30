/*
    This file is part of SSL.

    Copyright 2018 TO COMPLETE

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
class SlowStriker : public RobotBehavior
{
private:
  ConsignFollower* follower;
  double tempo;
  rhoban_geometry::Point striking_point;
  int robot_to_pass_id;
  Vision::Team robot_to_pass_team;

public:
  SlowStriker(Ai::AiData& ai_data);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  virtual Control control() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;
  void declare_point_to_strik(rhoban_geometry::Point point);
  void declare_robot_to_pass(int id, Vision::Team team = Vision::Team::Ally);
  virtual ~SlowStriker();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
