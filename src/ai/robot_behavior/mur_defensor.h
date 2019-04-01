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
namespace robot_behavior
{
class MurDefensor : public RobotBehavior
{
private:
  int mur_robot_id_;
  int mur_nb_robot_;
  ConsignFollower* follower_;

public:
  MurDefensor(ai::AiData& ai_data, bool fixed_consign_follower_without_repsecting_authorized_location_bool = 0);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);
  void declareMurRobotId(int id, int mur_nb_robots);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
  virtual ~MurDefensor();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
