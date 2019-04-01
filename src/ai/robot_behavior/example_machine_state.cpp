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

#include "example_machine_state.h"
#include <math/vector2d.h>
#include <debug.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
ExampleMachineState::ExampleMachineState(ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower_(Factory::fixedConsignFollower(ai_data)), machine(ai_data, ai_data)
{
  machine.addState(StateName::wait_pass,
                    [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      DEBUG("WAIT PASS");
                      this->printBallInfo();
                    });
  machine.addState(StateName::strike);

  machine.addEdge(EdgeName::can_strike, StateName::wait_pass, StateName::strike,  //,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     DEBUG("IS is_closed_to_the_ball " << this->isClosedToTheBall());
                     return this->isClosedToTheBall();
                   },
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     DEBUG("IS Can_striker FUAZHeuh");
                     this->printBallInfo();
                   });

  machine.addEdge(EdgeName::strike_is_finished, StateName::strike, StateName::wait_pass,
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     DEBUG("strike_is_finished");
                     return not(this->isClosedToTheBall());
                   },
                   [this](const ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     this->printBallInfo2();
                   });
  machine.addInitState(StateName::wait_pass);

  machine.start();

  machine.exportToFile("/tmp/toto.dot");
}

void ExampleMachineState::update(double time, const ai::Robot& robot, const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  machine.run();

  follower_->avoidTheBall(true);
  // follower->set_following_position(target_position, target_rotation);
  follower_->update(time, robot, ball);
}

Control ExampleMachineState::control() const
{
  Control ctrl = follower_->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

ExampleMachineState::~ExampleMachineState()
{
  delete follower_;
}

rhoban_ssl::annotations::Annotations ExampleMachineState::getAnnotations() const
{
  return follower_->getAnnotations();
}

bool ExampleMachineState::isClosedToTheBall() const
{
  DEBUG("is_closed fcttt" << (normSquare(this->linearPosition() - this->ballPosition()) < 0.05));
  return normSquare(this->linearPosition() - this->ballPosition()) < 0.05;
}

void ExampleMachineState::printArePass()
{
  DEBUG("ARE PASSING");
};

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
