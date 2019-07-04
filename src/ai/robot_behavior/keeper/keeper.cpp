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

#include "keeper.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <math/intersection.h>
#include "../navigation_inside_the_field.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
rhoban_geometry::Point Keeper::calculateGoalPosition(const rhoban_geometry::Point& ball_position,
                                                     const Vector2d& right_pole, const Vector2d& left_pole,
                                                     double keeper_radius)
{
  rhoban_geometry::Point defender_position = rhoban_geometry::centerOfConeIncircle(
      ball_position, vector2point(right_pole), vector2point(left_pole), keeper_radius);
  return defender_position;
}

Keeper::Keeper()
  : Keeper::Keeper(
        rhoban_geometry::Point(-Data::get()->field.field_length_ / 2.0, Data::get()->field.goal_width_ / 2.0),
        rhoban_geometry::Point(-Data::get()->field.field_length_ / 2.0, -Data::get()->field.goal_width_ / 2.0),
        rhoban_geometry::Point(-Data::get()->field.field_length_ / 2.0, 0.0) + ai::Config::waiting_goal_position,
        Data::get()->field.penalty_area_depth_, ai::Config::robot_radius)
{
}

Keeper::Keeper(const rhoban_geometry::Point& left_post_position, const rhoban_geometry::Point& right_post_position,
               const rhoban_geometry::Point& waiting_goal_position, double penalty_radius, double keeper_radius)
  : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
  left_post_position_ = left_post_position;
  right_post_position_ = right_post_position;
  waiting_goal_position_ = waiting_goal_position;
  goal_center_ = (left_post_position + right_post_position) / 2.0;
  penalty_radius_ = penalty_radius;
  keeper_radius_ = keeper_radius;
  NavigationInsideTheField* foll =
      dynamic_cast<NavigationInsideTheField*>(follower_);  // PID modification to be as responsive as Barthez
  foll->setTranslationPid(12, 0.0001, 0);
  foll->setOrientationPid(1.0, 0, 0);
}

void Keeper::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  annotations_.clear();

  /*int nb_points = 3;
  future_ball_positions_.clear();
  for (int i = 0; i < nb_points; i++)
  {
    future_ball_positions_.push_back(ball.getMovement().linearPosition(time + i * 0.2));
  }*/

  rhoban_geometry::Point left_post_position = left_post_position_;
  rhoban_geometry::Point right_post_position = right_post_position_;

  double OFFSET_GOAL = ai::Config::robot_radius * 1.5;
  double HYST = 0.10;

  rhoban_geometry::Point target_position;
  double target_rotation;
  rhoban_geometry::Point predicted_intersection_point;

  const rhoban_geometry::Point& predicted_ball_position = ball.getMovement().linearPosition(time + 3.0);
  // test if the ball hits the back

  if (predicted_ball_position.getX() < left_post_position.getX())
  {
    // finding the impact point

    int hyst_sign = 1;

    if (defensive_approach_ == 0)
    {
      hyst_sign = -1;
    }

    double post_offset = 0.7 + HYST * hyst_sign;
    const rhoban_geometry::Point new_left_post_position =
        left_post_position + rhoban_geometry::Point(OFFSET_GOAL, post_offset);
    const rhoban_geometry::Point new_right_post_position =
        right_post_position + rhoban_geometry::Point(OFFSET_GOAL, -post_offset);

    const rhoban_geometry::Segment predicted_ball_segment =
        rhoban_geometry::Segment(predicted_ball_position, ballPosition());
    const rhoban_geometry::Segment our_goal_segment =
        rhoban_geometry::Segment(new_left_post_position, new_right_post_position);

    bool do_they_intersect =
        segmentIntersection(predicted_ball_segment, our_goal_segment, predicted_intersection_point);

    if (do_they_intersect == true)
    {
      defensive_approach_ = 1;
      annotations_.addCross(predicted_intersection_point.x, predicted_intersection_point.y, "blue");
    }
    else
    {
      defensive_approach_ = 0;
    }
  }
  else
  {
    defensive_approach_ = 0;
  }

  if (defensive_approach_ == 0)
  {
    rhoban_geometry::Point new_goal_center =
        Data::get()->field.goalCenter(Ally) + rhoban_geometry::Point(OFFSET_GOAL, 0.0);

    rhoban_geometry::Point protect_position = ballPosition();
    if (ballPosition().getX() < Data::get()->field.goalCenter(Ally).getX())
    {
      protect_position = rhoban_geometry::Point(Data::get()->field.goalCenter(Ally).getX(), ballPosition().getY());
    }

    Vector2d ball_goal_vector = new_goal_center - protect_position;
    double dist_goal_vector = ball_goal_vector.norm();
    if (dist_goal_vector < 0.0001)
    {
      dist_goal_vector = 0.0001;
    }
    ball_goal_vector = ball_goal_vector / dist_goal_vector;

    target_rotation = detail::vec2angle(-ball_goal_vector);

    double goal_radius = 0.5;

    target_position = new_goal_center - ball_goal_vector * goal_radius;
  }
  else if (defensive_approach_ == 1)
  {
    if (predicted_intersection_point.getY() > left_post_position.getY())
    {
      target_position = rhoban_geometry::Point(predicted_intersection_point.getX(), left_post_position.getY());
    }
    else if (predicted_intersection_point.getY() < right_post_position.getY())
    {
      target_position = rhoban_geometry::Point(predicted_intersection_point.getX(), right_post_position.getY());
    }
    else
    {
      target_position = predicted_intersection_point;
    }

    Vector2d target_ball_vector = ballPosition() - target_position;
    target_rotation = detail::vec2angle(target_ball_vector);
  }

  follower_->setFollowingPosition(target_position, target_rotation);
  follower_->avoidTheBall(false);

  follower_->update(time, robot, ball);
}

Control Keeper::control() const
{
  Control ctrl = follower_->control();
  ctrl.chip_kick = true;
  return ctrl;
}

Keeper::~Keeper()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Keeper::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  /*
std::string annotations_text;

  if (defensive_approach_ == 0)
  {
    annotations_text = "Arc";
  }
  else
  {
    annotations_text = "Dash";
  }*/
  // annotations.addText(annotations_text, linearPosition().getX() + 0.15, linearPosition().getY() + 0.60,
  // "red");
  /*
    for (uint i = 0; i < future_ball_positions_.size(); i++)
    {
      annotations.addCross(future_ball_positions_[i].x, future_ball_positions_[i].y, "red", false);
    }*/

  annotations.addAnnotations(annotations_);
  annotations.addAnnotations(follower_->getAnnotations());
  return annotations;
}  // namespace robot_behavior

}  // namespace robot_behavior
}  // namespace rhoban_ssl
