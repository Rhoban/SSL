/*
    This file is part of SSL.
    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)
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

#include "../robot_behavior.h"
#include "../factory.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
class Keeper : public RobotBehavior
{
private:
  double FORWARD_DISTANCIATION_FROM_GOAL_CENTER = 0.05;

  Box goalkeeper_zone_;
  bool isInsideGoalKeeperZone(const rhoban_geometry::Point& point);

  rhoban_geometry::Point placeBetweenGoalCenterAndBall(const rhoban_geometry::Point& ball_position);

private:
  // PositionFollower follower(); TODO : to remove if not necessary

  rhoban_ssl::annotations::Annotations annotations_;

  ConsignFollower* follower_;

  rhoban_geometry::Point left_post_position_;
  rhoban_geometry::Point right_post_position_;
  rhoban_geometry::Point goal_center_;
  rhoban_geometry::Point waiting_goal_position_;

  double keeper_radius_;
  double penalty_radius_;

  int defensive_approach_ = 0;  // 0 arc-de-cercle, 1 dash
  const double OFFSET_GOAL = ai::Config::robot_radius * 1.5;
  const double HYST = 0.10;

  std::vector<rhoban_geometry::Point> future_ball_positions_;

  static rhoban_geometry::Point calculateGoalPosition(const rhoban_geometry::Point& ballPosition,
                                                      const Vector2d& right_pole, const Vector2d& left_pole,
                                                      double keeper_radius_);

public:
  Keeper();

  Keeper(const rhoban_geometry::Point& left_post_position, const rhoban_geometry::Point& right_post_position,
         const rhoban_geometry::Point& waiting_goal_position, double penalty_radius, double keeper_radius);

  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~Keeper();
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
