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
namespace strategy
{
/**
 * @brief Strategy used to stop all robots by setting the behabior "DoNothing" to all of them.
 */
class Halt : public Strategy
{
public:
  Halt();

  int minRobots() const;
  int maxRobots() const;
  virtual GoalieNeed needsGoalie() const;

  static const std::string name;

  void start(double time);
  void stop(double time);

  void assignBehaviorToRobots(std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior,
                              double time, double dt);
  virtual ~Halt();

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;
};

};  // namespace strategy
};  // namespace rhoban_ssl
