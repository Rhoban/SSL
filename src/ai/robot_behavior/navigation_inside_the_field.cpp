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

#include "navigation_inside_the_field.h"
#include <rhoban_geometry/segment.h>
#include <physic/constants.h>
#include <physic/collision.h>
#include <math/box.h>
#include <math/intersection.h>

#include <debug.h>
#include <core/print_collection.h>
#include <core/collection.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
NavigationInsideTheField::NavigationInsideTheField(double time, double dt, bool forceInsidePenaltyArea)
  : ConsignFollower()
  , need_to_avoid_the_ball_(true)
  , saving_ball_radius_avoidance_(ai::Config::robot_radius)
  , following_position_was_updated_(true)
  , position_follower_(time, dt)
  , target_position_(0.0, 0.0)
  , target_angle_(0.0)
  , deviation_position_(0.0, 0.0)
  , forceInsidePenaltyArea_(forceInsidePenaltyArea)
{
}

void NavigationInsideTheField::setFollowingPosition(const rhoban_geometry::Point& position_to_follow,
                                                    const ContinuousAngle& target_angle)
{
  following_position_was_updated_ = true;
  this->target_position_ = position_to_follow;
  this->target_angle_ = target_angle;
}

void NavigationInsideTheField::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  update_control(time, robot, ball);
}

void NavigationInsideTheField::update_control(double time, const data::Robot& robot, const data::Ball& ball)
{
  annotations_.clear();
  if ((Data::get()->ai_data.force_ball_avoidance) && (ballPosition().x > -2.5) &&
      ((ballPosition().x - robot.getMovement().linearPosition(time).x) <= 0)  // most common case, robot come from

  )
  {
    this->position_follower_.setRadiusAvoidanceForTheBall(ai::Config::robot_radius + ai::Config::ball_radius +
                                                          ai::Config::rules_avoidance_distance);
    this->avoidTheBall(true);
  }
  else
  {
    this->position_follower_.setRadiusAvoidanceForTheBall(saving_ball_radius_avoidance_);
    this->avoidTheBall(need_to_avoid_the_ball_);
  }

  double marge = 2 * ai::Config::ball_radius;
  if (following_position_was_updated_)
  {
    // Box cropped_field(
    //     field_SW() + Vector2d( marge, marge ),
    //     field_NE() - Vector2d( marge, marge)
    // );
    // Box cropped_field(
    //     field_SW(),
    //     field_NE()
    // );
    // Trying agressive margins
    Box cropped_field(Data::get()->field.getSW() - Vector2d(marge, marge),
                      Data::get()->field.getNE() + Vector2d(marge, marge));
    float radius_margin_factor = 2.0;
    Box opponent_penalty = Data::get()->field.getPenaltyArea(Opponent).increase(ai::Config::robot_radius);
    Box ally_penalty = Data::get()->field.getPenaltyArea(Ally).increase(ai::Config::robot_radius);

    Box opponent_penalty_large =
        Data::get()->field.getPenaltyArea(Opponent).increase(ai::Config::ball_radius * radius_margin_factor);
    Box ally_penalty_large =
        Data::get()->field.getPenaltyArea(Ally).increase(ai::Config::ball_radius * radius_margin_factor);

    rhoban_geometry::Point robot_position = linearPosition();
    double error = ai::Config::ball_radius * radius_margin_factor;

    if (opponent_penalty.isInside(robot_position))
    {
      // If we're in their penalty
      deviation_position_ = rhoban_geometry::Point(opponent_penalty.getSW().getX() - error, robot_position.getY());
    }
    else if (not(forceInsidePenaltyArea_) and not(isGoalie()) and ally_penalty.isInside(robot_position))
    {
      // If we're in our penalty
      deviation_position_ = rhoban_geometry::Point(ally_penalty.getNE().getX() + error, robot_position.getY());
    }
    else
    {
      if (cropped_field.isInside(vector2point(target_position_)))
      {
        // Normal case, the goal position is in the field
        deviation_position_ = vector2point(target_position_);
      }
      else
      {
        // Changing the target position to match the closest segment
        cropped_field.closestSegmentIntersection(robot_position, vector2point(target_position_), deviation_position_);
      }
      // Here, deviation_position should be inside the field, but it could still be in a penalty area.
      if (not(isGoalie()) and ally_penalty.isInside(deviation_position_))
      {
        ally_penalty_large.closestSegmentIntersection(robot_position, vector2point(deviation_position_),
                                                      deviation_position_);
        if (not(cropped_field.isInside(deviation_position_)))
        {
          deviation_position_ = deviation_position_ + Vector2d(Data::get()->field.penalty_area_depth + error, 0.0);
        }
      }
      else if (opponent_penalty.isInside(deviation_position_))
      {
        opponent_penalty_large.closestSegmentIntersection(robot_position, vector2point(deviation_position_),
                                                          deviation_position_);
        if (not(cropped_field.isInside(deviation_position_)))
        {
          deviation_position_ = deviation_position_ - Vector2d(Data::get()->field.penalty_area_depth + error, 0.0);
        }
      }
    }

    if ((not(robot.is_goalie) && Data::get()->field.penalty_areas[Ally].isInside(deviation_position_)) ||
        Data::get()->field.penalty_areas[Opponent].isInside(deviation_position_))
    {
      DEBUG("Damn, deviation_position is still inside a penalty (this is a bug)...");
    }

    this->position_follower_.setFollowingPosition(deviation_position_, target_angle_);
    following_position_was_updated_ = false;
  }
  position_follower_.update(time, robot, ball);
}

Control NavigationInsideTheField::control() const
{
  return position_follower_.control();
}

void NavigationInsideTheField::setTranslationPid(double kp, double ki, double kd)
{
  position_follower_.setTranslationPid(kp, ki, kd);
}
void NavigationInsideTheField::setOrientationPid(double kp, double ki, double kd)
{
  position_follower_.setOrientationPid(kp, ki, kd);
}

void NavigationInsideTheField::avoidTheBall(bool value)
{
  need_to_avoid_the_ball_ = value;
  position_follower_.avoidTheBall(value);
}

void NavigationInsideTheField::avoidAlly(bool value)
{
  position_follower_.avoidAlly(value);
}

void NavigationInsideTheField::avoidOpponent(bool value)
{
  position_follower_.avoidOpponent(value);
}

void NavigationInsideTheField::avoidRobot(int id, bool value)
{
  position_follower_.avoidRobot(id, value);
}

void NavigationInsideTheField::setLimits(double translation_velocity_limit, double rotation_velocity_limit,
                                         double translation_acceleration_limit, double rotation_acceleration_limit)
{
  position_follower_.setLimits(translation_velocity_limit, rotation_velocity_limit, translation_acceleration_limit,
                               rotation_acceleration_limit);
}

rhoban_ssl::annotations::Annotations NavigationInsideTheField::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  if (normSquare(deviation_position_ - target_position_) > 0.001)
  {
    annotations.addArrow(deviation_position_, target_position_, "yellow");
  }
  annotations.addAnnotations(position_follower_.getAnnotations());
  annotations.addAnnotations(annotations_);
  // annotations.addBox( opponent_penalty_area(), "red" );
  // annotations.addBox( ally_penalty_area(), "red" );
  return annotations;
}

void NavigationInsideTheField::setRadiusAvoidanceForTheBall(double radius)
{
  position_follower_.setRadiusAvoidanceForTheBall(radius);
  saving_ball_radius_avoidance_ = radius;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
