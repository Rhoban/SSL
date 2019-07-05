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

#pragma once

#include <robot_behavior/robot_behavior.h>
#include <robot_behavior/factory.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
class PokeBall : public RobotBehavior
{
private:
  ConsignFollower* follower_;
  rhoban_ssl::annotations::Annotations annotations_;
  bool ready_to_kick_ = false;
  double kick_power_ = 0.3;
  rhoban_geometry::Point poke_direction_ = Data::get()->field.goalCenter(Opponent);

public:
  PokeBall();

  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   * @brief set the direction of the poke (kick)
   *
   * @param poke_direction
   * rhoban_geometry::Point for the poke direction. Default: opponent goalCenter.
   */
  void setPokeDirection(rhoban_geometry::Point poke_direction);

  /**
   * @brief set kick power of the poke
   *
   * @param kick_power
   * value between 0 and 1
   */
  void setKickPower(double kick_power);

  virtual ~PokeBall();
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
