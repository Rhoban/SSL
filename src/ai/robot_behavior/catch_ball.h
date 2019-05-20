/*
    This file is part of SSL.

    Copyright 2018 TO COMPLETE

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

#include "robot_behavior.h"
#include "factory.h"
#include <math/vector2d.h>


namespace rhoban_ssl
{
namespace robot_behavior
{
class CatchBall : public RobotBehavior
{
private:
  /**
   * @brief Distance needed to catch the ball (On the ball's movement vector).
   */
  // const double dist_to_front_of_ball_;

  /**
   * @brief Distance needed between ball and robot (perpendicular to the ball's movement vector) while the robot is behind the ball (On the ball's movement vector).
   */
  // const double dist_next_to_ball_;

  /**
   * @brief Is the robot positionned to catch the ball?
   */
  bool robot_can_catch_;

  /**
   * @brief Is the ball/robot distance null, and have they stopped moving?
   */
  bool ball_was_caught_;
  
  ConsignFollower* follower_;

public:

  /**
   * @brief Constructor
   * @param
   * @see ai::AiData
   */
  
  CatchBall(ai::AiData& ai_data);

  
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~CatchBall();

};
} // namespace robot_behavior
} // namespace rhoban_ssl

  //TODO: Verifying that the robot can catch the ball without leaving the field.
  //If ball is not moving, directly moves to ball, while looking at it.
