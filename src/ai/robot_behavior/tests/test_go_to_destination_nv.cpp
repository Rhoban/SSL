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

#include "test_go_to_destination_nv.h"
#include <math/vector2d.h>
#include <debug.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
TestGoToDestinationNV::TestGoToDestinationNV() : RobotBehavior()
{
}

void TestGoToDestinationNV::setAngularVelocity(const ContinuousAngle& angular_velocity)
{
  this->angular_velocity_ = angular_velocity;
}

void TestGoToDestinationNV::setLinearVelocity(const Vector2d& linear_velocity)
{
  this->linear_velocity_ = linear_velocity;
}

void TestGoToDestinationNV::setDestination(const int x, const int y, const int ang)
{
  destination_x_ = x;
  destination_y_ = y;
  destination_ang_ = ang;
}

bool TestGoToDestinationNV::getDestinationReached()
{
  return destination_is_reached_;
}

void TestGoToDestinationNV::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  if (state_ == 0)
  {
    init_x_ = robot.electronics.xpos;
    init_y_ = robot.electronics.ypos;
    init_ang_ = robot.electronics.ang;
    state_ = 1;
  }
  else if (state_ == 1)
  {
    int16_t xpos = robot.electronics.xpos;
    int16_t ypos = robot.electronics.ypos;
    int16_t ang = robot.electronics.ang;
    std::cout << "Odometry (x,y,ang): " << xpos << " " << ypos << " " << ang << std::endl;

    if (((xpos - init_x_) >= destination_x_) and ((ypos - init_y_) >= destination_y_) and
        ((ang - init_ang_) >= destination_ang_))
    {
      destination_is_reached_ = true;
    }
  }
}

Control TestGoToDestinationNV::control() const
{
  Control ctrl(false);
  if (!destination_is_reached_)
  {
    ctrl.linear_velocity = linear_velocity_;
    ctrl.angular_velocity = angular_velocity_;
  }
  else
  {
    ctrl.linear_velocity = Vector2d(0, 0);
    ctrl.angular_velocity = 0;
  }
  return ctrl;
}

TestGoToDestinationNV::~TestGoToDestinationNV()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations TestGoToDestinationNV::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace tests
}  // namespace robot_behavior
}  // namespace rhoban_ssl
