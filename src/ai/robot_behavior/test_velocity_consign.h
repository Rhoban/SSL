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

#include "robot_behavior.h"
#include "factory.h"

namespace rhoban_ssl
{
namespace Robot_behavior
{
class Test_velocity_consign : public RobotBehavior
{
private:
  ConsignFollower* follower;
  RhobanSSLAnnotation::Annotations annotations;

  double period;
  double last_time;
  int cpt;

  Vector2d linear_velocity;
  ContinuousAngle angular_velocity;

public:
  Test_velocity_consign(Ai::AiData& ai_data);

  virtual void update(double time, const Ai::Robot& robot, const Ai::Ball& ball);

  static Test_velocity_consign* get_W_movement(Ai::AiData& ai_data, double velocity);
  static Test_velocity_consign* get_E_movement(Ai::AiData& ai_data, double velocity);
  static Test_velocity_consign* get_N_movement(Ai::AiData& ai_data, double velocity);
  static Test_velocity_consign* get_S_movement(Ai::AiData& ai_data, double velocity);
  static Test_velocity_consign* get_NW_movement(Ai::AiData& ai_data, double velocity);
  static Test_velocity_consign* get_NE_movement(Ai::AiData& ai_data, double velocity);
  static Test_velocity_consign* get_SW_movement(Ai::AiData& ai_data, double velocity);
  static Test_velocity_consign* get_SE_movement(Ai::AiData& ai_data, double velocity);

  void set_linear_velocity(const Vector2d& velocity);
  void set_angular_velocity(const ContinuousAngle& angle);

  virtual Control control() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  virtual ~Test_velocity_consign();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
