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
namespace robot_behavior
{
class Degageur : public RobotBehavior
{
private:
  rhoban_geometry::Point point_to_pass_;
  int robot_to_pass_id_;
  Team robot_to_pass_team_;
  bool needKick_;

  ConsignFollower* follower_;

public:
  Degageur();

  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);
  void declarePointToPass(rhoban_geometry::Point point);
  void declareRobotToPass(int robot_id, Team team = Ally);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~Degageur();
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
