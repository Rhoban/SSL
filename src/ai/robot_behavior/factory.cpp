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
namespace robot_behavior
{
ConsignFollower* Factory::fixedConsignFollower(ai::AiData& ai_data, const rhoban_geometry::Point& position,
                                                 const ContinuousAngle& angle, bool ignore_the_ball)
{
  return Factory::fixedConsignFollowerWithoutRepsectingAuthorizedLocation(ai_data, position, angle,
                                                                                ignore_the_ball);
}

ConsignFollower* Factory::fixedConsignFollowerWithoutRepsectingAuthorizedLocation(
    ai::AiData& ai_data, const rhoban_geometry::Point& position, const ContinuousAngle& angle, bool ignore_the_ball)
{
  // A_star_path* follower = new A_star_path(ai_data, ai_data.time, ai_data.dt);
  NavigationInsideTheField* follower = new NavigationInsideTheField(ai_data, ai_data.time, ai_data.dt);
  // Navigation_with_obstacle_avoidance* follower = new Navigation_with_obstacle_avoidance(ai_data, ai_data.time,
  // ai_data.dt);
  // PositionFollower* follower = new PositionFollower(ai_data, ai_data.time, ai_data.dt);
  follower->setTranslationPid(ai_data.constants.p_translation, ai_data.constants.i_translation,
                                ai_data.constants.d_translation);
  follower->setOrientationPid(ai_data.constants.p_orientation, ai_data.constants.i_orientation,
                                ai_data.constants.d_orientation);
  follower->setLimits(ai_data.constants.translation_velocity_limit, ai_data.constants.rotation_velocity_limit,
                       ai_data.constants.translation_acceleration_limit, ai_data.constants.rotation_acceleration_limit);
  follower->setFollowingPosition(position, angle);
  follower->avoidTheBall(not(ignore_the_ball));
  return follower;
}

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
