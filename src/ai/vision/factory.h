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
#pragma once

#include "robot_position_filter.h"
#include <math/position.h>

namespace rhoban_ssl
{
namespace vision
{
struct TimedPosition
{
  double time_;
  Position position_;
  bool orientation_is_defined_;
  TimedPosition();
};

class Factory
{
public:
  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  filter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
         const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
         vision::PartOfTheField part_of_the_field_used);

  static TimedPosition filter(RobotDetection** robots);
};

};  // namespace vision
};  // namespace rhoban_ssl
