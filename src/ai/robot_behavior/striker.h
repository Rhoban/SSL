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
class Striker : public RobotBehavior
{
private:
  bool use_custom_vector_;
  rhoban_geometry::Point striking_point_;
  ConsignFollower* follower_;

public:
  Striker(ai::AiData& ai_data);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  void declarePointToStrike(rhoban_geometry::Point point);

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~Striker();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
