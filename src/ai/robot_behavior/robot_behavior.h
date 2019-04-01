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
#include <ai_data.h>
#include <annotations/annotations.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
class RobotBehavior : public GameInformations
{
protected:
  const ai::Robot* robot_ptr_;
  double birthday_;
  double last_update_;
  std::string name_;

  Vector2d robot_linear_position_;
  ContinuousAngle robot_angular_position_;
  Vector2d robot_linear_velocity_;
  ContinuousAngle robot_angular_velocity_;

  ai::AiData& ai_data_;

public:
  RobotBehavior(ai::AiData& ia_data);

  double age() const;
  bool isBorn() const;
  void setBirthday(double birthday);

  void updateTimeAndPosition(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball) = 0;
  virtual Control control() const = 0;

  //
  // This function is used to draw annotations in the viewer.
  // You can use it to print what you want.
  //
  // For example :
  //
  //
  //  RhobanSSLAnnotation::Annotations get_annotations() const{
  //      RhobanSSLAnnotation::Annotations annotations;
  //      static double d = 0;
  //      d += 0.01;
  //
  //      annotations.addCircle(3, 3, 1, "cyan");
  //      annotations.addArrow(0, 0, cos(d), sin(d)*2, "magenta", true);
  //      return annotations;
  //  }
  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  const ai::Robot& robot() const;

  rhoban_geometry::Point linearPosition() const;
  ContinuousAngle angularPosition() const;
  bool isGoalie() const;

  bool infraRed() const;
};

namespace detail
{
double vec2angle(Vector2d direction);
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
