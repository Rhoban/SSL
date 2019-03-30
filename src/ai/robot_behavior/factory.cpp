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

#include "factory.h"

#include "position_follower.h"
#include "navigation_with_obstacle_avoidance.h"
#include "navigation_inside_the_field.h"
#include "a_star_path.h"

namespace rhoban_ssl
{
namespace Robot_behavior
{
ConsignFollower* Factory::fixed_consign_follower(Ai::AiData& ai_data, const rhoban_geometry::Point& position,
                                                 const ContinuousAngle& angle, bool ignore_the_ball)
{
  return Factory::fixed_consign_follower_without_repsecting_authorized_location(ai_data, position, angle,
                                                                                ignore_the_ball);
}

ConsignFollower* Factory::fixed_consign_follower_without_repsecting_authorized_location(
    Ai::AiData& ai_data, const rhoban_geometry::Point& position, const ContinuousAngle& angle, bool ignore_the_ball)
{
  // A_star_path* follower = new A_star_path(ai_data, ai_data.time, ai_data.dt);
  Navigation_inside_the_field* follower = new Navigation_inside_the_field(ai_data, ai_data.time, ai_data.dt);
  // Navigation_with_obstacle_avoidance* follower = new Navigation_with_obstacle_avoidance(ai_data, ai_data.time,
  // ai_data.dt);
  // PositionFollower* follower = new PositionFollower(ai_data, ai_data.time, ai_data.dt);
  follower->set_translation_pid(ai_data.constants.p_translation, ai_data.constants.i_translation,
                                ai_data.constants.d_translation);
  follower->set_orientation_pid(ai_data.constants.p_orientation, ai_data.constants.i_orientation,
                                ai_data.constants.d_orientation);
  follower->set_limits(ai_data.constants.translation_velocity_limit, ai_data.constants.rotation_velocity_limit,
                       ai_data.constants.translation_acceleration_limit, ai_data.constants.rotation_acceleration_limit);
  follower->set_following_position(position, angle);
  follower->avoid_the_ball(not(ignore_the_ball));
  return follower;
}

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
