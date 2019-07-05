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
#include <math/lines.h>

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

  rhoban_geometry::Point old_ball_position = ball.movement_sample[9].linear_position;
  Vector2d ball_trajectory = ball.getMovement().linearVelocity(time);

  if (ball_trajectory.norm() == 0)
  {
    follower_->setFollowingPosition(Data::get()->field.goalCenter(Ally), 0.0);
    follower_->update(time, robot, ball);
    return;
  }

  rhoban_geometry::Point pole_left = Data::get()->field.getGoal(Ally).pole_left_;
  rhoban_geometry::Point pole_right = Data::get()->field.getGoal(Ally).pole_right_;

  double robot_diameter = ai::Config::robot_radius * 2;
  rhoban_geometry::Point offset(robot_diameter, 0.0);
  double distanciation = (robot_diameter + FORWARD_DISTANCIATION_FROM_GOAL_CENTER);
  rhoban_geometry::Point limit_left_position_on_trajectory = pole_left + offset;
  rhoban_geometry::Point limit_right_position_on_trajectory = pole_right - offset;

  double chord_square_norm = limit_left_position_on_trajectory.getDist(limit_right_position_on_trajectory) *
                             limit_left_position_on_trajectory.getDist(limit_right_position_on_trajectory);

  double circle_radius_center_of_the_trajectory =
      (chord_square_norm / (8 *distanciation)) + distanciation / 2;

  double distance_goal_center_to_circle_center =
      std::abs(circle_radius_center_of_the_trajectory - distanciation);

  rhoban_geometry::Point circle_center_of_the_trajectory =
      Data::get()->field.getGoal(Ally).goal_center_ -
      rhoban_geometry::Point(distance_goal_center_to_circle_center, 0.0); 

  rhoban_geometry::Circle robot_trajectory(circle_center_of_the_trajectory, circle_radius_center_of_the_trajectory);

  goalkeeper_zone_ = Box(limit_right_position_on_trajectory, limit_left_position_on_trajectory + offset);
  //// END OF COMPUTATION

  std::vector<rhoban_geometry::Point> intersections = rhoban_geometry::getIntersectionLineWithCircle(
      old_ball_position, old_ball_position + ball_trajectory, robot_trajectory);

  if (intersections.size() == 0)
  {
    // on se place par rapport au centre du goal et de la balle
    ContinuousAngle target_angular_position = vector2angle(ball_trajectory * -1); // ça c'est de la DAUBE
    follower_->setFollowingPosition(placeBetweenGoalCenterAndBall((old_ball_position)), target_angular_position);
  }
  else if (intersections.size() == 1)
  {
    if (goalkeeper_zone_.is_inside(intersections.at(0)))
    {
      ContinuousAngle target_angular_position = vector2angle(ball_trajectory * -1); // ça c'est de la DAUBE
      follower_->setFollowingPosition(intersections.at(0), target_angular_position);
    }
    else
    {
      // on se place par rapport au centre du goal et de la balle
      ContinuousAngle target_angular_position = vector2angle(ball_trajectory * -1); // ça c'est de la DAUBE
      follower_->setFollowingPosition(placeBetweenGoalCenterAndBall((old_ball_position)), target_angular_position);
    }
  }
  else if (intersections.size() == 2)
  {
    if (goalkeeper_zone_.is_inside(intersections.at(0)))
    {
      if (goalkeeper_zone_.is_inside((intersections.at(1))))
      {
        // ici on choisit le plus proche de la balle
        double dist1 = intersections.at(0).getDist(old_ball_position);
        double dist2 = intersections.at(1).getDist(old_ball_position);
        if (dist1 > dist2)
        {
          ContinuousAngle target_angular_position = vector2angle(ball_trajectory * -1); // ça c'est de la DAUBE
          follower_->setFollowingPosition(intersections.at(1), target_angular_position);
        }
        else
        {
          ContinuousAngle target_angular_position = vector2angle(ball_trajectory * -1); // ça c'est de la DAUBE 
          follower_->setFollowingPosition(intersections.at(0), target_angular_position);
        }
      }
      else
      {
        ContinuousAngle target_angular_position = vector2angle(ball_trajectory * -1); // ça c'est de la DAUBE
        follower_->setFollowingPosition(intersections.at(0), target_angular_position);
      }
    }
    else
    {
      if (goalkeeper_zone_.is_inside((intersections.at(1))))
      {
        ContinuousAngle target_angular_position = vector2angle(ball_trajectory * -1);
        follower_->setFollowingPosition(intersections.at(1), target_angular_position);
      }
      else
      {
        // on se place par rapport au centre du goal et de la balle
        ContinuousAngle target_angular_position = vector2angle(ball_trajectory * -1);
        follower_->setFollowingPosition(placeBetweenGoalCenterAndBall((old_ball_position)), target_angular_position);
      }
    }
  }
  else
  {
    throw "INVALID INTERSECTION COMPUTATION";
  }

  follower_->avoidTheBall(false);

  follower_->update(time, robot, ball);
}

// bool Keeper::isInsideGoalKeeperZone(const rhoban_geometry::Point& point)
// {
//   return goalkeeper_zone_.is_inside(point);
// } NOT USED

rhoban_geometry::Point Keeper::placeBetweenGoalCenterAndBall(const rhoban_geometry::Point& ball_position)
{
  rhoban_geometry::Point goal_center = Data::get()->field.goalCenter(Ally);

  double dist_ball_goal_center = goal_center.getDist(ball_position);
  if (dist_ball_goal_center < 0.001) {
    dist_ball_goal_center = 0.001;
  }
  double distance_between_ball_and_goal_on_x_axis = std::abs(goal_center.getX() - ball_position.getX());
  double distance_between_ball_and_meridian = ball_position.getY();
  double cos_theta = distance_between_ball_and_goal_on_x_axis / dist_ball_goal_center;
  int pos = (distance_between_ball_and_meridian > 0.0) ? 1 : -1; // assign 1 or -1 depending on the upper or lower half of the field, considering the y = 0 dividing line
  int signe = ((Data::get()->field.getGoal(Ally).pole_left_.getY() / pos) > 0.0) ? 1 : -1; //assign 1 or -1 whether the ball is the same side as the left pole or not

  double position_to_take_y =
      signe * cos_theta *
      (Data::get()->field.getGoal(Ally).pole_left_.getY() - Data::get()->field.getGoal(Ally).pole_right_.getY()) / 2;
  rhoban_geometry::Point position_to_take = rhoban_geometry::Point(ai::Config::robot_radius * 2, position_to_take_y);
  return position_to_take;
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
