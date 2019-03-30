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
#include <core/machine_state.h>

namespace rhoban_ssl
{
namespace Robot_behavior
{
class Example_machine_state : public RobotBehavior
{
private:
  ConsignFollower* follower;
  RhobanSSLAnnotation::Annotations annotations;

public:
  struct state_name
  {
    static const constexpr char* wait_pass = "wait_pass";
    static const constexpr char* strike = "strike";
  };
  struct edge_name
  {
    static const constexpr char* can_strike = "can_strike";
    static const constexpr char* strike_is_finished = "strike_is_finished";
  };

  bool is_closed_to_the_ball() const;

  void print_are_pass()
  {
    DEBUG("ARE PASSING");
  }
  void print_ball_info()
  {
    DEBUG("INFO");
  }
  void print_ball_info2()
  {
    DEBUG("INFO2");
  }

  typedef construct_machine_state_infrastructure<std::string, ai::AiData, ai::AiData> machine_state_infrastructure;

private:
  machine_state_infrastructure::MachineState machine;

public:
  Example_machine_state(ai::AiData& ai_data);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  virtual RhobanSSLAnnotation::Annotations get_annotations() const;

  virtual ~Example_machine_state();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
