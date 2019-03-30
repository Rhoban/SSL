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
namespace Robot_behavior
{
Example_machine_state::Example_machine_state(Ai::AiData& ai_data)
  : RobotBehavior(ai_data), follower(Factory::fixed_consign_follower(ai_data)), machine(ai_data, ai_data)
{
  machine.add_state(state_name::wait_pass,
                    [this](const Ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      DEBUG("WAIT PASS");
                      this->print_ball_info();
                    });
  machine.add_state(state_name::strike);

  machine.add_edge(edge_name::can_strike, state_name::wait_pass, state_name::strike,  //,
                   [this](const Ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     DEBUG("IS is_closed_to_the_ball " << this->is_closed_to_the_ball());
                     return this->is_closed_to_the_ball();
                   },
                   [this](const Ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     DEBUG("IS Can_striker FUAZHeuh");
                     this->print_ball_info();
                   });

  machine.add_edge(edge_name::strike_is_finished, state_name::strike, state_name::wait_pass,
                   [this](const Ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     DEBUG("strike_is_finished");
                     return not(this->is_closed_to_the_ball());
                   },
                   [this](const Ai::AiData& data, unsigned int run_number, unsigned int atomic_run_number) {
                     this->print_ball_info2();
                   });
  machine.add_init_state(state_name::wait_pass);

  machine.start();

  machine.export_to_file("/tmp/toto.dot");
}

void Example_machine_state::update(double time, const Ai::Robot& robot, const Ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position(time, robot, ball);

  machine.run();

  follower->avoid_the_ball(true);
  // follower->set_following_position(target_position, target_rotation);
  follower->update(time, robot, ball);
}

Control Example_machine_state::control() const
{
  Control ctrl = follower->control();
  // ctrl.spin = true; // We active the dribler !
  ctrl.kick = false;
  return ctrl;
}

Example_machine_state::~Example_machine_state()
{
  delete follower;
}

RhobanSSLAnnotation::Annotations Example_machine_state::get_annotations() const
{
  return follower->get_annotations();
}

bool Example_machine_state::is_closed_to_the_ball() const
{
  DEBUG("is_closed fcttt" << (norm_square(this->linear_position() - this->ballPosition()) < 0.05));
  return norm_square(this->linear_position() - this->ballPosition()) < 0.05;
};

}  // namespace Robot_behavior
}  // namespace rhoban_ssl
