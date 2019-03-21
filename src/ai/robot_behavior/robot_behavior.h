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

#ifndef __ROBOT_BEHAVIOR__ROBOT_BEHAVIOR__H__
#define __ROBOT_BEHAVIOR__ROBOT_BEHAVIOR__H__

#include <game_informations.h>
#include <control/robot_control_with_position_following.h>
#include <control/robot_control_with_curve.h>
#include <control/control.h>
#include <rhoban_utils/angle.h>
#include <AiData.h>
#include <annotations/Annotations.h>

namespace RhobanSSL
{
namespace Robot_behavior
{
class RobotBehavior : public GameInformations
{
protected:
  const Ai::Robot* robot_ptr;
  double birthday;
  double lastUpdate;
  std::string name;

  Vector2d robot_linear_position;
  ContinuousAngle robot_angular_position;
  Vector2d robot_linear_velocity;
  ContinuousAngle robot_angular_velocity;

  Ai::AiData& ai_data;

public:
  RobotBehavior(Ai::AiData& ia_data);

  double age() const;
  bool is_born() const;
  void set_birthday(double birthday);

  void update_time_and_position(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball) = 0;
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
  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  const Ai::Robot& robot() const;

  rhoban_geometry::Point linear_position() const;
  ContinuousAngle angular_position() const;
  bool is_goalie() const;

  bool infra_red() const;
};

namespace detail
{
double vec2angle(Vector2d direction);
};

};  // namespace Robot_behavior
};  // namespace RhobanSSL

#endif
