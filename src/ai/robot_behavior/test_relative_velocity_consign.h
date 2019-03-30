/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

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

namespace rhoban_ssl
{
namespace Robot_behavior
{
class Test_relative_velocity_consign : public RobotBehavior
{
private:
  Control relative_control;

public:
  Test_relative_velocity_consign(ai::AiData& ai_data);
  virtual ~Test_relative_velocity_consign();

  void set_linear_velocity(const Vector2d& linear_velocity);
  void set_angular_velocity(const ContinuousAngle& angular_velocity);

  // RobotBehavior interface
public:
  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);
  virtual Control control() const;
  virtual rhoban_ssl::annotations::Annotations get_annotations() const;

  // Tests
public:
  static Test_relative_velocity_consign* get_movement_angular_velocity_only(ai::AiData& ai_data,
                                                                            double angular_velocity);
  static Test_relative_velocity_consign* get_movement_linear_velocity_only(ai::AiData& ai_data,
                                                                           Vector2d linear_velocity);
  static Test_relative_velocity_consign* get_movement_angular_and_linear_velocity(ai::AiData& ai_data,
                                                                                  Vector2d linear_velocity,
                                                                                  double angular_velocity);
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
