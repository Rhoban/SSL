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

#include "navigation_with_obstacle_avoidance.h"
#include <rhoban_geometry/segment.h>
#include <physic/constants.h>
#include <physic/collision.h>
#include <data/computed_data.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
NavigationWithObstacleAvoidance::NavigationWithObstacleAvoidance(double time, double dt)
  : ConsignFollower()
  , ignore_the_ball_(false)
  , ignore_robot_()
  , ball_radius_avoidance_(ai::Config::robot_radius)
  , position_follower_(time, dt)
  , position_follower_avoidance_(time, dt)
  , target_position_(0.0, 0.0)
  , target_angle_(0.0)
{
}

void NavigationWithObstacleAvoidance::setFollowingPosition(const rhoban_geometry::Point& position_to_follow,
                                                           const ContinuousAngle& target_angle)
{
  this->position_follower_.setFollowingPosition(position_to_follow, target_angle);
  this->target_position_ = position_to_follow;
  this->target_angle_ = target_angle;
  this->target_angle_ = this->robot_angular_position_;
  this->target_angle_.setToNearest(target_angle);
}

void NavigationWithObstacleAvoidance::determineTheClosestObstacle()
{
  // We determine the closest obstaclte in term of time collision.

  Control ctrl = position_follower_.control();

  min_time_collision_ = -1;
  closest_robot_ = -1;
  second_closest_robot_ = -1;
  std::list<std::pair<int, double> > collisions_with_ctrl =
      data::CollisionComputing::getCollisions(robot().id, ctrl.linear_velocity);
  assert(ai::Config::security_acceleration_ratio > ai::Config::obstacle_avoidance_ratio);
  double ctrl_velocity_norm = ctrl.linear_velocity.norm();
  double time_to_stop =
      ctrl_velocity_norm / (ai::Config::obstacle_avoidance_ratio * ai::Config::translation_acceleration_limit);

  for (const std::pair<int, double>& collision : collisions_with_ctrl)
  {
    if (ignore_robot_[collision.first])
    {
      continue;
    }
    double time_before_collision = collision.second;
    if (time_before_collision <= time_to_stop and ctrl_velocity_norm > EPSILON_VELOCITY)
    {
      if ((min_time_collision_ == -1) or (min_time_collision_ > time_before_collision))
      {
        min_time_collision_ = time_before_collision;
        closest_robot_ = collision.first;
      }
    }
  }

  // second closest robot
  double mdist = 9999;
  for (unsigned int j = 0; j < Data::get()->all_robots.size(); j++)
  {
    data::Robot* r = Data::get()->all_robots.at(j).second;
    if (r->isActive() == false)
      continue;
    if (r->id != robot().id && r->id != (uint)closest_robot_)
    {
      rhoban_geometry::Point rpos = r->getMovement().linearPosition(r->getMovement().lastTime());
      Vector2d v = (rpos - robot().getMovement().linearPosition(robot().getMovement().lastTime()));
      double d = v.norm();
      if (d < mdist)
      {
        mdist = d;
        second_closest_robot_ = r->id;
        second_closest_distance_ = d;
      }
    }
  }

  ball_is_the_obstacle_ = false;
  if (not(ignore_the_ball_))
  {
    double radius_error = ai::Config::radius_security_for_collision;
    std::pair<bool, double> collision =
        collisionTime(ai::Config::robot_radius, robot().getMovement().linearPosition(robot().getMovement().lastTime()),
                      ctrl.linear_velocity, ball_radius_avoidance_,
                      ball().getMovement().linearPosition(ball().getMovement().lastTime()),
                      ball().getMovement().linearVelocity(ball().getMovement().lastTime()), radius_error);
    if (collision.first)
    {
      double time_before_collision = collision.second;
      if (time_before_collision <= time_to_stop and ctrl_velocity_norm > EPSILON_VELOCITY)
      {
        if ((min_time_collision_ == -1) or (min_time_collision_ > time_before_collision))
        {
          min_time_collision_ = time_before_collision;
          ball_is_the_obstacle_ = true;
        }
      }
    }
  }
}

void NavigationWithObstacleAvoidance::computeTheRadiusOfLimitCycle()
{
  // Is yet constructed at construction
  assert(ai::Config::radius_security_for_collision < ai::Config::radius_security_for_avoidance);

#if 1
  if (ball_is_the_obstacle_)
  {
    assert(not(ignore_the_ball_));  // Normally determine_the_closest_obstacle() set ball_is_the_obstacle to false when
                                    // we ignore the ball
    radius_of_limit_cycle_ =
        ai::Config::ball_radius + ai::Config::robot_radius + ai::Config::radius_security_for_avoidance;
  }
  else
  {
    if (robot().getMovement().linearVelocity(Data::get()->ai_data.time).norm() <
        ai::Config::translation_velocity_limit / 4.0)
    {
      radius_of_limit_cycle_ = 2 * ai::Config::robot_radius;  // + ai_data.constants.radius_security_for_avoidance;
    }
    else
    {
      radius_of_limit_cycle_ = 2 * ai::Config::robot_radius + ai::Config::radius_security_for_avoidance;
    }
  }
#endif
}

void NavigationWithObstacleAvoidance::convertCycleDirectionToLinearAndAngularVelocity()
{
  // avoidance_control.kick = false;
  // avoidance_control.ignore = false;

  // Control follower_control = position_follower.control();
  // avoidance_control.angular_velocity = follower_control.angular_velocity;
  // avoidance_control.linear_velocity = limit_cycle_direction*(
  //     follower_control.linear_velocity.norm()/limit_cycle_direction.norm()
  // );

  assert(limit_cycle_direction_.norm() != 0.0);

  rhoban_geometry::Point pos = linearPosition() + limit_cycle_direction_ / (limit_cycle_direction_.norm()) * 1.0;

  double dist =
      linearPosition().getDist(Data::get()->all_robots[closest_robot_].second->getMovement().linearPosition(time()));

  // avoid the problem where 2 allies bots infinitely avoid themselves until they leave the field like two small
  // dragonfly.
  if (Data::get()->all_robots[closest_robot_].first == Ally && dist < INFINITE_DODGING_PREVENTION &&
      robot().getMovement().linearVelocity(time()).norm() <=
          Data::get()->all_robots[closest_robot_].second->getMovement().linearVelocity(time()).norm())
  {
    pos = linearPosition();
  }
  position_follower_avoidance_.setFollowingPosition(pos, target_angle_);
}

void NavigationWithObstacleAvoidance::computeTheLimitCycleDirectionForObstacle(
    const rhoban_geometry::Point& obstacle_linear_position, const Vector2d& obstacle_linear_velocity)
{
  obstacle_point_of_view_.robot_linear_position =
      vector2point(robot_linear_position_ - Vector2d(obstacle_linear_position));
  obstacle_point_of_view_.target_linear_position = vector2point(target_position_ - Vector2d(obstacle_linear_position));

  /////////////////////////////////////////////////////////////////
  // We compute the sens of avoidance rotatiion
  /////////////////////////////////////////////////////////////////
  sign_of_avoidance_rotation_ = 1.0;  // TODO

  // data::Robot& obstacle = *(Data::get()->all_robots[closest_robot_].second);
  Vector2d obstacle_to_goal = vector2point(target_position_) - obstacle_linear_position;
  Vector2d current_to_goal = vector2point(target_position_) - linearPosition();
  double angle = vector2angle(current_to_goal).value() - vector2angle(obstacle_to_goal).value();

  // if(second_closest_distance<0.2) //TODO parameter
  // {
  //   ai::Robot & second_obstacle = *( ai_data.all_robots[second_closest_robot].second );
  //   Vector2d second_obstacle_to_goal=vector2point(target_position)-second_obstacle.;
  // }

  const rhoban_geometry::Point& s = obstacle_point_of_view_.robot_linear_position;

  double XX = s.getX() * s.getX();
  double YY = s.getY() * s.getY();

  double avoidance_convergence = ai::Config::coefficient_to_increase_avoidance_convergence;
  if (Data::get()->all_robots[closest_robot_].first == Data::get()->all_robots[robot().id].first)
  {
    // sign_of_avoidance_rotation = 1.0;
    avoidance_convergence = (XX + YY) * (XX + YY);
  }

  {
    if (angle < 0.0)
    {
      sign_of_avoidance_rotation_ = 1;
    }
    else
      sign_of_avoidance_rotation_ = -1;
  }

  /////////////////////////////////////////////////////////////////
  // We compute now the limit cycle rotation
  /////////////////////////////////////////////////////////////////
  double delta_radius;

  if ((XX + YY) == 0.0)
    delta_radius = 0.5;
  else
    delta_radius = (radius_of_limit_cycle_ * radius_of_limit_cycle_ - XX - YY) / (XX + YY) * avoidance_convergence;
  obstacle_point_of_view_.limit_cycle_direction =
      Vector2d(sign_of_avoidance_rotation_ * s.getY() + s.getX() * delta_radius,
               -sign_of_avoidance_rotation_ * s.getX() + s.getY() * delta_radius);

  limit_cycle_direction_ = obstacle_point_of_view_.limit_cycle_direction + obstacle_linear_velocity;
}

void NavigationWithObstacleAvoidance::computeTheLimitCycleDirectionForRobot()
{
  /////////////////////////////////////////////////////////////////
  // We change the frame from absolute to frame relative to obstacle
  /////////////////////////////////////////////////////////////////
  data::Robot& obstacle = *(Data::get()->all_robots[closest_robot_].second);
  rhoban_geometry::Point obstacle_linear_position = obstacle.getMovement().linearPosition(time());
  Vector2d obstacle_linear_velocity = obstacle.getMovement().linearVelocity(time());

  computeTheLimitCycleDirectionForObstacle(obstacle_linear_position, obstacle_linear_velocity);
}

void NavigationWithObstacleAvoidance::computeTheLimitCycleDirectionForBall()
{
  /////////////////////////////////////////////////////////////////
  // We change the frame from absolute to frame relative to obstacle
  /////////////////////////////////////////////////////////////////
  rhoban_geometry::Point obstacle_linear_position = ball().getMovement().linearPosition(time());
  Vector2d obstacle_linear_velocity = ball().getMovement().linearVelocity(time());

  computeTheLimitCycleDirectionForObstacle(obstacle_linear_position, obstacle_linear_velocity);
}

void NavigationWithObstacleAvoidance::computeTheLimitCycleDirection()
{
  if (ball_is_the_obstacle_)
  {
    assert(not(ignore_the_ball_));  // Normally determine_the_closest_obstacle() set ball_is_the_obstacle to false when
                                    // we ignore the ball
    computeTheLimitCycleDirectionForBall();
  }
  else
  {
    computeTheLimitCycleDirectionForRobot();
  }
}

void NavigationWithObstacleAvoidance::update(double time, const data::Robot& robot, const data::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);
  // Now
  //  this->robot_linear_position
  //  this->ball_position
  //  this->robot_angular_position
  //  this->robot()
  // are all avalaible

  updateControl(time, robot, ball);
}

void NavigationWithObstacleAvoidance::updateControl(double time, const data::Robot& robot, const data::Ball& ball)
{
  position_follower_.update(time, robot, ball);  // We use the future command to predict collision
  determineTheClosestObstacle();
  if (min_time_collision_ >= 0)
  {
    computeTheRadiusOfLimitCycle();
    computeTheLimitCycleDirection();
    convertCycleDirectionToLinearAndAngularVelocity();

    position_follower_avoidance_.update(time, robot, ball);
  }
}

Control NavigationWithObstacleAvoidance::control() const
{
  if (min_time_collision_ >= 0)
  {
    return position_follower_avoidance_.control();
  }
  else
  {
    return position_follower_.control();
  }
}

void NavigationWithObstacleAvoidance::setTranslationPid(double kp, double ki, double kd)
{
  position_follower_.setTranslationPid(kp, ki, kd);
}
void NavigationWithObstacleAvoidance::setOrientationPid(double kp, double ki, double kd)
{
  position_follower_.setOrientationPid(kp, ki, kd);
}

void NavigationWithObstacleAvoidance::avoidTheBall(bool value)
{
  ignore_the_ball_ = not(value);
}
void NavigationWithObstacleAvoidance::avoidAlly(bool value)
{
  for (int i = 0; i < ai::Config::NB_OF_ROBOTS_BY_TEAM; i++)
  {
    ignore_robot_[i] = not(value);
  }
}
void NavigationWithObstacleAvoidance::avoidOpponent(bool value)
{
  for (int i = ai::Config::NB_OF_ROBOTS_BY_TEAM; i < 2 * ai::Config::NB_OF_ROBOTS_BY_TEAM; i++)
  {
    ignore_robot_[i] = not(value);
  }
}
void NavigationWithObstacleAvoidance::avoidRobot(int id, bool value)
{
  ignore_robot_[id] = not(value);
}

void NavigationWithObstacleAvoidance::setLimits(double translation_velocity_limit, double rotation_velocity_limit,
                                                double translation_acceleration_limit,
                                                double rotation_acceleration_limit)
{
  position_follower_.setLimits(translation_velocity_limit, rotation_velocity_limit, translation_acceleration_limit,
                               rotation_acceleration_limit);
}

rhoban_ssl::annotations::Annotations NavigationWithObstacleAvoidance::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  //    annotations.addCircle( linear_position(), radius_of_limit_cycle );

  if (min_time_collision_ >= 0)
  {
    annotations.addArrow(linearPosition(),
                         linearPosition() + limit_cycle_direction_ * (limit_cycle_direction_.norm()) * 10, "red");
    if (closest_robot_ == -1)
    {
      annotations.addCircle(ball().getMovement().linearPosition(Data::get()->ai_data.time), radius_of_limit_cycle_);
    }
    else
    {
      annotations.addCircle(
          Data::get()->all_robots.at(closest_robot_).second->getMovement().linearPosition(Data::get()->ai_data.time),
          radius_of_limit_cycle_);
    }
    annotations.addAnnotations(position_follower_avoidance_.getAnnotations());
  }
  else
  {
    annotations.clear();
    annotations.addAnnotations(position_follower_.getAnnotations());
  }

  return annotations;
}

void NavigationWithObstacleAvoidance::setRadiusAvoidanceForTheBall(double radius)
{
  ball_radius_avoidance_ = radius;
}

double NavigationWithObstacleAvoidance::getRadiusAvoidanceForTheBall()
{
  return ball_radius_avoidance_;
}

}  // namespace robot_behavior
}  // namespace rhoban_ssl
