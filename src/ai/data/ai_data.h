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

#include <map>
#include <com/ai_commander.h>

namespace rhoban_ssl
{
namespace data
{
class AiData
{
public:
  AiData();

  /**
   * @brief time shift with vision in seconds
   */
  double time_shift_with_vision;

  /**
   * @brief time in seconds
   */
  double time;

  /**
   * @brief dt with last loop in seconds
   */
  double dt;

  // This field is used by rhobot_behavior::Navigation_inside_the_field.
  bool force_ball_avoidance;

  /**
   * @brief Collision_times_table
   * @note come from ai_data
   */
  typedef std::map<std::pair<int, int>, double> Collision_times_table;
  Collision_times_table table_of_collision_times_;
};

}  // namespace data
}  // namespace rhoban_ssl
