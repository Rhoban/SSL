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
#include "factory.h"

namespace rhoban_ssl
{
namespace Robot_behavior
{
class Goalie : public RobotBehavior
{
private:
  // PositionFollower follower(); TODO : to remove if not necessary

  RhobanSSLAnnotation::Annotations annotations;

  ConsignFollower* follower;

  Vector2d left_post_position;
  Vector2d right_post_position;
  Vector2d goal_center;
  rhoban_geometry::Point waiting_goal_position;

  double goalie_radius;
  double penalty_rayon;
  int defensive_approach;

  static rhoban_geometry::Point calculate_goal_position(const rhoban_geometry::Point& ballPosition,
                                                        const Vector2d& poteau_droit, const Vector2d& poteau_gauche,
                                                        double goalie_radius);

public:
  Goalie(Ai::AiData& ai_data);

  Goalie(Ai::AiData& ai_data, const Vector2d& left_post_position, const Vector2d& right_post_position,
         const rhoban_geometry::Point& waiting_goal_position, double penalty_rayon, double goalie_radius, double time,
         double dt);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  std::vector<rhoban_geometry::Point> future_ball_positions;

  virtual Control control() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  virtual ~Goalie();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
