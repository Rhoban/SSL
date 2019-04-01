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

#include "robot_behavior.h"
#include "navigation_with_obstacle_avoidance.h"
#include "position_follower.h"
#include <ai_data.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
/*
 * This is an implementation of the article :
 * "Orbital Obstavle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class AStarPath : public ConsignFollower
{
private:
  NavigationWithObstacleAvoidance navigation_;

  Vector2d target_position_;
  ContinuousAngle target_angle_;

public:
  AStarPath(ai::AiData& ai_data, double time, double dt);

protected:
  void update_control(double time, const ai::Robot& robot, const ai::Ball& ball);

public:
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  void setTranslationPid(double kp, double ki, double kd);
  void setOrientationPid(double kp, double ki, double kd);

  void setLimits(double translation_velocity_limit, double rotation_velocity_limit,
                 double translation_acceleration_limit, double rotation_acceleration_limit);

  virtual void setFollowingPosition(const rhoban_geometry::Point& position_to_follow, const ContinuousAngle& angle);
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
