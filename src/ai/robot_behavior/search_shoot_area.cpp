/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#include "search_shoot_area.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <debug.h>
#include <core/print_collection.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
SearchShootArea::SearchShootArea(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
  , obstructed_view_(-1)
  , period_(3)
  , last_time_changement_(0)
  , follower_(Factory::fixedConsignFollower(ai_data))
  , well_positioned(false)
{
  p1_ = Vector2d(opponentGoalCenter()) + rhoban_geometry::Point(-1, 2);
  p2_ = Vector2d(centerMark()) + rhoban_geometry::Point(1, -2);
}

void SearchShootArea::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  annotations_.clear();

  const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time);
  // Vector2d opponent_goal_robot_vector = robot_position - opponent_goal_center();

  std::pair<rhoban_geometry::Point, double> results = GameInformations::findGoalBestMove(robot_position);

  annotations_.addArrow(robot_position, results.first, "red");

  double seuil = 0.4;

  double pos_x = robot_position.getX();
  double pos_y = robot_position.getY();
  Vector2d ball_robot_vector = ballPosition() - robot_position;
  ContinuousAngle target_rotation = vector2angle(ball_robot_vector);

  // DEBUG("obstructed_view AFTER : " << obstructed_view);
  if ((results.second > seuil) && pos_x <= std::max(p1_.x, p2_.x) && pos_x > std::min(p1_.x, p2_.x) &&
      pos_y <= std::max(p1_.y, p2_.y) && pos_y > std::min(p1_.y, p2_.y))
  {
    // DEBUG( "robot_position : " << robot_position );
    target_position_ = robot_position;
    well_positioned = true;
    // DEBUG( "target_position : " << target_position );
  }
  else
  {
    if (time > last_time_changement_ + period_)
    {
      std::uniform_real_distribution<double> distribution_x(p1_.x, p2_.x);
      std::uniform_real_distribution<double> distribution_y(p1_.y, p2_.y);
      target_position_ = rhoban_geometry::Point(distribution_x(generator_), distribution_y(generator_));
      last_time_changement_ = time;
      well_positioned = false;
    }
  }

  // target_position = robot_position;
  // target_position = robot_position;

  annotations_.addCross(target_position_.x, target_position_.y);

  follower_->avoidTheBall(false);
  follower_->setFollowingPosition(target_position_, target_rotation);
  follower_->update(time, robot, ball);
}

Control SearchShootArea::control() const
{
  Control ctrl = follower_->control();
  return ctrl;
}

void SearchShootArea::declareArea(rhoban_geometry::Point p1, rhoban_geometry::Point p2)
{
  this->p1_ = p1;
  this->p2_ = p2;
}

SearchShootArea::~SearchShootArea()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations SearchShootArea::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  annotations.addAnnotations(this->annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
