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

#include "mobile.h"

#include <structs.h>

namespace rhoban_ssl
{
namespace data
{
class Robot : public Mobile
{
public:
  Robot();

  int id;
  bool is_goalie;

  /**
   * @brief true if inside field and in vision
   */
  bool is_valid;

  //todo default state
  struct packet_robot electronics;

  /**
   * @brief infraRed returns true if the infrared barrier of the robot detects somethings
   * @return bool : infrared barrier detection
   */
  bool infraRed() const;
};

}  // namespace data
}  // namespace rhoban_ssl
