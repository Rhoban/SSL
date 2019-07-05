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

#include "kick_measure.h"
#include <math/vector2d.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace test
{
KickMeasure::KickMeasure(double kick_power)
  : RobotBehavior()
  , follower_(Factory::fixedConsignFollower())
  , kick_power_(kick_power)
  , started_(false)
  , countdown_(5)
  , measures_done_(false)
  , reached_dist_(-1)
  , max_speed_(-1)
  , printed_(false)

{
}

void KickMeasure::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update position from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  annotations_.clear();

  if (!started_)
  {
    started_ = true;
    std::cout << ("BEGIN KICK MEASURE with a power of :") << std::endl;
    std::cout << "         " << (kick_power_ * 100) << " %" << std::endl;
    std::cout << ("put the ball in front of the kicker at center before the countdown end !") << std::endl;
    std::cout << (countdown_) << std::endl;
    last_time_ = time;
  }
  else
  {
    if (countdown_ > 0)
    {
      if (time - last_time_ >= 1.0)
      {
        last_time_ = time;
        countdown_--;
        std::cout << (countdown_) << std::endl;
        if (countdown_ == 0)
        {
          ball_first_position_ = ballPosition();
        }
      }
    }
    else
    {
      // main behavior:
      if (!measures_done_)
      {
        double speed = ball.getMovement().linearVelocity(time).norm();
        std::cout << ("instant speed : ") << speed << std::endl;
        if (speed > max_speed_)
        {
          max_speed_ = speed;
        }
        if (speed <= 0.05 && (time - last_time_) > 2.0)
        {
          measures_done_ = true;
        }
      }
      else
      {
        if (!printed_)
        {
          reached_dist_ = ball_first_position_.getDist(ballPosition());
          if (ballPosition().x > (Data::get()->field.field_length / 2) ||
              ballPosition().x < -(Data::get()->field.field_length / 2) ||
              ballPosition().y > (Data::get()->field.field_width / 2) ||
              ballPosition().y < -(Data::get()->field.field_width / 2))
          {
            reached_dist_ = -1;
          }
          std::cout << ("RESULTS :") << std::endl;
          std::cout << ("Reached distance :") << std::endl;
          if (reached_dist_ == -1)
          {
            std::cout << ("ball exited the field") << std::endl;
          }
          else
          {
            std::cout << "         " << reached_dist_ << " m" << std::endl;
          }
          std::cout << ("Maximum ball speed :") << std::endl;
          std::cout << "         " << max_speed_ << " m/s" << std::endl;
          printed_ = true;
        }
      }
    }
  }

  follower_->setFollowingPosition(linearPosition(), 0);
  follower_->avoidTheBall(false);
  follower_->update(time, robot, ball);
}

Control KickMeasure::control() const
{
  Control ctrl = follower_->control();
  ctrl.kick_power = kick_power_;
  ctrl.charge = true;
  if (started_ && countdown_ == 0)
  {
    ctrl.kick = true;
  }
  else
  {
    ctrl.kick = false;
  }

  return ctrl;
}

KickMeasure::~KickMeasure()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations KickMeasure::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace test
}  // namespace robot_behavior
}  // namespace rhoban_ssl