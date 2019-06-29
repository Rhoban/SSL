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

#include "pvc_goalie.h"
#include <math/tangents.h>
#include <math/vector2d.h>
#include <math/intersection.h>
#include "navigation_inside_the_field.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
rhoban_geometry::Point Goalie::calculateGoalPosition(const rhoban_geometry::Point& ball_position,
                                                     const Vector2d& poteau_droit, const Vector2d& poteau_gauche,
                                                     double goalie_radius)
{
  rhoban_geometry::Point defender_position = rhoban_geometry::centerOfConeIncircle(
      ball_position, vector2point(poteau_droit), vector2point(poteau_gauche), goalie_radius);
  return defender_position;
}

Goalie::Goalie()
  : Goalie::Goalie(Vector2d(-Data::get()->field.field_length_ / 2.0, Data::get()->field.goal_width_ / 2.0),
                   Vector2d(-Data::get()->field.field_length_ / 2.0, -Data::get()->field.goal_width_ / 2.0),
                   rhoban_geometry::Point(-Data::get()->field.field_length_ / 2.0, 0.0) +
                       ai::Config::waiting_goal_position,
                   Data::get()->field.penalty_area_depth_, ai::Config::robot_radius, Data::get()->ai_data.time,
                   Data::get()->ai_data.dt)
{
}

Goalie::Goalie(const Vector2d& left_post_position, const Vector2d& right_post_position,
               const rhoban_geometry::Point& waiting_goal_position, double penalty_rayon, double goalie_radius,
               double time, double dt)
  : RobotBehavior(), follower_(Factory::fixedConsignFollower())
{
  this->left_post_position_ = left_post_position;
  this->right_post_position_ = right_post_position;
  this->waiting_goal_position_ = waiting_goal_position;
  this->goal_center_ = (left_post_position + right_post_position) / 2.0;
  this->penalty_rayon_ = penalty_rayon;
  this->goalie_radius_ = goalie_radius;
  defensive_approach_ = 0;  // 0 arc-de-cercle, 1 dash
  NavigationInsideTheField* foll = dynamic_cast<NavigationInsideTheField*>(follower_);
  foll->setTranslationPid(3.0, 0.0001, 0);
  foll->setOrientationPid(1.0, 0, 0);
}

void Goalie::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->robot_angular_position
  // are all avalaible

  annotations_.clear();

  future_ball_positions.clear();
  // int nb_points = 10;
  // for (int i=0; i < nb_points; i++) {
  //     future_ball_positions.push_back( ball.get_movement().linear_position( time + i * 0.2 ) );
  // }

  rhoban_geometry::Point left_post_position =
      rhoban_geometry::Point(-Data::get()->field.field_length_ / 2.0, Data::get()->field.goal_width_ / 2.0);
  rhoban_geometry::Point right_post_position =
      rhoban_geometry::Point(-Data::get()->field.field_length_ / 2.0, -Data::get()->field.goal_width_ / 2.0);

  double offset_goal = ai::Config::robot_radius * 1.5;
  double hyst = 0.10;

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

    double post_offset = 0.7 + hyst * hyst_sign;
    const rhoban_geometry::Point new_left_post_position =
        left_post_position + rhoban_geometry::Point(offset_goal, post_offset);
    const rhoban_geometry::Point new_right_post_position =
        right_post_position + rhoban_geometry::Point(offset_goal, -post_offset);

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
        Data::get()->field.goalCenter(Ally) + rhoban_geometry::Point(offset_goal, 0.0);

    rhoban_geometry::Point protect_position = ballPosition();
    if (ballPosition().getX() < Data::get()->field.goalCenter(Ally).getX())
    {
      protect_position = rhoban_geometry::Point(Data::get()->field.goalCenter(Ally).getX(), ballPosition().getY());
    }

    Vector2d ball_goal_vector = new_goal_center - protect_position;
    ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();

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

Control Goalie::control() const
{
  Control ctrl = follower_->control();
  ctrl.chip_kick = true;
  return ctrl;
}

Goalie::~Goalie()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations Goalie::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations_local;
  annotations_local.addAnnotations(annotations_);
  std::string annotations_text;
  if (robot_ptr_)
  {
    if (defensive_approach_ == 0)
    {
      annotations_text = "Arc";
    }
    else
    {
      annotations_text = "Dash";
    }
    // annotations_local.addText(annotations_text, linearPosition().getX() + 0.15, linearPosition().getY() + 0.60,
    // "red");

    // DEBUG("nb_future_ball = " << future_ball_positions.size() );
    // for (int i = 0; i < future_ball_positions.size(); i++) {
    //     //DEBUG(future_ball_positions[i].x << " " << future_ball_positions[i].y );
    //     annotations_local.addCross(future_ball_positions[i].x, future_ball_positions[i].y, "red" );
    // }
  }
  annotations_local.addAnnotations(follower_->getAnnotations());
  return annotations_local;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
