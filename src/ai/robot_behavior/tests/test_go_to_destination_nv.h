/*
    This file is part of SSL.

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

#include "../factory.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
class TestGoToDestinationNV : public RobotBehavior
{
private:
  ConsignFollower* follower_;
  rhoban_ssl::annotations::Annotations annotations_;

  std::vector<rhoban_geometry::Point> path_;
  int path_index_;

  const double REACH_RADIUS = 0.1;
  Vector2d linear_velocity_;
  ContinuousAngle angular_velocity_;
  bool destination_is_reached_ = false;

  int destination_x_;
  int destination_y_;
  int destination_ang_;

  int init_x_;
  int init_y_;
  int init_ang_;

  int state_ = 0;

public:
  TestGoToDestinationNV();

  void setLinearVelocity(const Vector2d& velocity);
  void setAngularVelocity(const ContinuousAngle& angle);
  void setDestination(const int x, const int y, const int ang);
  bool getDestinationReached();

  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~TestGoToDestinationNV();
};

};  // namespace tests
};  // namespace robot_behavior
};  // namespace rhoban_ssl
