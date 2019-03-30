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

#include "strategy.h"
#include <string>

namespace rhoban_ssl
{
namespace Strategy
{
class Halt : public Strategy
{
public:
  Halt(Ai::AiData& ai_data);

  int min_robots() const;
  int max_robots() const;
  virtual Goalie_need needs_goalie() const;

  static const std::string name;

  void start(double time);
  void stop(double time);

  void assign_behavior_to_robots(
      std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt);
  virtual ~Halt();

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;
};

};  // namespace Strategy
};  // namespace rhoban_ssl
