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
namespace robot_behavior
{
class ExampleMachineState : public RobotBehavior
{
private:
  ConsignFollower* follower_;
  rhoban_ssl::annotations::Annotations annotations_;

public:
  struct StateName
  {
    static const constexpr char* wait_pass = "wait_pass";
    static const constexpr char* strike = "strike";
  };
  struct EdgeName
  {
    static const constexpr char* can_strike = "can_strike";
    static const constexpr char* strike_is_finished = "strike_is_finished";
  };

  bool isClosedToTheBall() const;

  void printArePass();
  void printBallInfo()
  {
    DEBUG("INFO");
  }
  void printBallInfo2()
  {
    DEBUG("INFO2");
  }

  typedef construct_machine_state_infrastructure<std::string, ai::AiData, ai::AiData> MachineStateInfrastructure;

private:
  MachineStateInfrastructure::MachineState machine;

public:
  ExampleMachineState(ai::AiData& ai_data);

  virtual void update(double time, const ai::Robot& robot, const ai::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual ~ExampleMachineState();
};

};  // namespace Robot_behavior
};  // namespace rhoban_ssl
