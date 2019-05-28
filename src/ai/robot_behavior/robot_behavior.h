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

#pragma once

#include <game_informations.h>
#include <control/robot_control_with_position_following.h>
#include <control/robot_control_with_curve.h>
#include <control/control.h>
#include <rhoban_utils/angle.h>
#include <data.h>
#include <annotations/annotations.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
/**
 *@brief In our IA model, Managers control Strategies wich control basic Behaviors.
 */
class RobotBehavior : public GameInformations
{
protected:
  /**
   *@brief Pointer to the robot on which we assign the behavior
   */
  const data::Robot* robot_ptr_;

  /**
   *@brief Time value, considered as the born date of the robot.
   */
  double birthday_;

  /**
   *@brief Time value, the current "date", updated at each call of updateTimeAndPosition(), if the function is not
   *overriden.
   */
  double last_update_;

  /**
   *@brief coordinates of the robot, updated at each call of updateTimeAndPosition(), if the function is not
   *overriden.
   */
  Vector2d robot_linear_position_;
  /**
   *@brief orientation of the robot, updated at each call of updateTimeAndPosition(), if the function is not
   *overriden.
   */
  ContinuousAngle robot_angular_position_;
  /**
   *@brief velocity of the robot, updated at each call of updateTimeAndPosition(), if the function is not
   *overriden.
   */
  Vector2d robot_linear_velocity_;
  /**
   *@brief angular velocity of the robot, updated at each call of updateTimeAndPosition(), if the function is not
   *overriden.
   */
  ContinuousAngle robot_angular_velocity_;

public:
  RobotBehavior();

  std::string name;
  double age() const;
  bool isBorn() const;
  void setBirthday(double birthday);

  /**
   * @brief update information about time and robot position and velocity
   *
   * do not override ! And call this function at the begining of each update() of the behavior.
   *
   * @param time : the time
   * @param robot : the robot reference
   * @param ball : the ball reference
   */
  void updateTimeAndPosition(double time, const data::Robot& robot, const data::Ball& ball);

  /**
   * @brief called regularly, its the main loop of execution of the behavior.
   *
   * @param time : the time
   * @param robot : the robot reference
   * @param ball : the ball reference
   */
  virtual void update(double time, const data::Robot& robot, const data::Ball& ball) = 0;

  /**
   * @brief Called frequently as update(), put code to setting Control of the robot, the class wich order movements,
   * kicker, ...
   * @return Control
   */
  virtual Control control() const = 0;

  /**
   * @brief This function is used to draw annotations in the viewer.
   * You can use it to print what you want.
   *
   * For example :
   *
   *
   *  RhobanSSLAnnotation::Annotations get_annotations() const{
   *      RhobanSSLAnnotation::Annotations annotations;
   *      static double d = 0;
   *      d += 0.01;
   *
   *      annotations.addCircle(3, 3, 1, "cyan");
   *      annotations.addArrow(0, 0, cos(d), sin(d)*2, "magenta", true);
   *      return annotations;
   *  }
   * @return annotations
   * @see rhoban_ssl::annotations::Annotations
   */
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  /**
   * @brief get robot on which this behavior is applied.
   * @return pointer to the robot
   */
  const data::Robot& robot() const;

  /**
   * @brief Other way to get robot position
   * @return coordinates
   */
  rhoban_geometry::Point linearPosition() const;

  /**
   * @brief Other way to get robot angle
   * @return angle
   */
  ContinuousAngle angularPosition() const;

  /**
   * @brief To know if the robot is the goalie
   * @return bool
   */
  bool isGoalie() const;

  /**
   * @brief To know if there is something inside the kicker. Because that will cut the IR barrier.
   * @return bool
   */
  bool infraRed() const;
};

namespace detail
{
double vec2angle(Vector2d direction);
};

};  // namespace robot_behavior
};  // namespace rhoban_ssl
