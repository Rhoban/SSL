/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 TO COMPLETE -> Gregwar

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

#include "ai_commander.h"
#include <debug.h>
#include <config.h>
#include <data.h>

#include <unistd.h>

namespace rhoban_ssl
{
namespace control
{
Commander::Commander() : real_(nullptr), sim_(nullptr)
{
  if (ai::Config::is_in_simulation)
  {
    if (sim_ == nullptr)
      sim_ = new SimClient();
  }
  else
  {
    if (real_ == nullptr)
    {
      real_ = new Master("/dev/ttyACM0", 1000000);
      /*
       * COULD HELP BUT NOT FOR SURE:
      for (int rid = 0; rid < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++rid)
        this->set(rid, true, 0, 0, 0);
      this->send();
      */
      usleep(1000);
    }
  }
  if (Data::get()->commander != nullptr)
  {
    throw "commander already set!";
  }
  Data::get()->commander = this;
}

Commander::~Commander()
{
  emergency();
  usleep(1000);
  if (sim_ != nullptr)
    delete sim_;

  if (real_ != nullptr)
    delete real_;

  assert(Data::get()->commander == this);

  Data::get()->commander = nullptr;
}

void Commander::set(uint8_t robot_id, bool enabled, double x_speed, double y_speed, double theta_speed, int kick,
                    float kick_power, bool spin, bool charge, bool tare_odom)
{
  assert(kick_power >= 0.0 && kick_power <= 1.0);

  if (robot_id >= MAX_ROBOTS)
    return;

  Command command;
  command.enabled = enabled;
  command.robot_id = robot_id;
  command.x_speed = x_speed;
  command.y_speed = y_speed;
  command.theta_speed = theta_speed;
  command.kick = kick;
  command.spin = spin;
  command.charge = charge;
  command.kick_power = kick_power;
  command.tare_odom = tare_odom;
  commands_.push_back(command);
}

void Commander::stopAll()
{
  for (int k = 0; k < 8; k++)
  {
    set(k, false, 0, 0, 0);
  }
}

void Commander::emergency()
{
  for (uint id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
  {
    auto& final_control = Data::get()->shared_data.final_control_for_robots[id];
    final_control.is_manually_controled_by_viewer = true;
    final_control.control.ignore = true;
    final_control.control.active = false;
  }

  this->stopAll();
  this->send();
}

void Commander::moveBall(double x, double y, double vx, double vy)
{
  if (ai::Config::is_in_simulation)
  {
    if (Data::get()->referee.allyOnPositiveHalf())
    {
      x *= -1;
    }
    sim_->moveBall(x, y, vx, vy);
  }
  else
  {
    // do nothing
  }
}

void Commander::moveRobot(bool ally, int id, double x, double y, double theta)
{
  static const bool turn_on = true;
  if (ai::Config::is_in_simulation)
  {
    if (Data::get()->referee.allyOnPositiveHalf())
    {
      x *= -1;
      theta += M_PI;
    }

    bool yellow = true;
    if ((ai::Config::we_are_blue && ally) || (!ally && !ai::Config::we_are_blue))
    {
      yellow = false;
    }
    sim_->moveRobot(yellow, id, x, y, theta * 180 / M_PI, turn_on);
  }
  else
  {
    // do nothing
  }
}

grSim_Packet Commander::convertToSimulationPacket(const Commander::Command& cmd)
{
  grSim_Packet packet;
  packet.mutable_commands()->set_isteamyellow(!ai::Config::we_are_blue);
  packet.mutable_commands()->set_timestamp(0.0);

  double factor = cmd.enabled ? 1 : 0;

  grSim_Robot_Command* simCommand = packet.mutable_commands()->add_robot_commands();

  double kick_x = 0;
  double kick_y = 0;

  // XXX: These values should be realistic depending on what we do on the real robot!
  if (cmd.enabled)
  {
    if (cmd.kick == 1)
    {
      kick_x = 6 * cmd.kick_power;
      kick_y = 0;
    }
    else if (cmd.kick == 2)
    {
      kick_x = 4 * cmd.kick_power;
      kick_y = 4 * cmd.kick_power;
    }
  }

  // Appending data
  simCommand->set_id(cmd.robot_id);
  simCommand->set_wheelsspeed(false);
  simCommand->set_veltangent(cmd.x_speed * factor);
  simCommand->set_velnormal(cmd.y_speed * factor);
  simCommand->set_velangular(cmd.theta_speed * factor);
  simCommand->set_kickspeedx(kick_x);
  simCommand->set_kickspeedz(kick_y);
  simCommand->set_spinner(cmd.enabled ? cmd.spin : false);

  return packet;
}

packet_master Commander::convertToRobotPacket(const Commander::Command& cmd)
{
  struct packet_master packet;
  if (cmd.enabled)
  {
    packet.actions = ACTION_ON;

    if (cmd.charge)
    {
      packet.actions |= ACTION_CHARGE;
    }

    if (cmd.kick == 1)
    {
      packet.actions |= ACTION_KICK1;
    }

    if (cmd.kick == 2)
    {
      packet.actions |= ACTION_KICK2;
    }

    if (cmd.spin)
    {
      packet.actions |= ACTION_DRIBBLE;
    }
    if (cmd.tare_odom)
    {
      packet.actions |= ACTION_TARE_ODOM;
    }
  }
  else
  {
    packet.actions = 0;
  }

  packet.kickPower = 100 * cmd.kick_power;

  if (cmd.tare_odom)
  {
    packet.t_speed = cmd.theta_speed * 10000;
  }
  else
  {
    packet.t_speed = cmd.theta_speed * 1000;
  }
  packet.x_speed = cmd.x_speed * 1000;
  packet.y_speed = cmd.y_speed * 1000;
  packet.t_speed = cmd.theta_speed * 1000;

  return packet;
}

void Commander::updateRobotsCommands()
{
  for (uint robot_id = 0; robot_id < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++robot_id)
  {
    Control& ctrl = Data::get()->shared_data.final_control_for_robots[robot_id].control;
    if (!Data::get()->robots[Ally][robot_id].infraRed() && !ai::Config::is_in_simulation)
    {
      ctrl.kick = false;
      ctrl.chip_kick = false;
      ctrl.kick_power = 0;
    }

    if (robot_id >= 8)
    {                      // HACK - becaus hardware doesn't support more than 8 robots
      continue;            // HACK
    }                      // HACK
    assert(robot_id < 8);  // HACK !
    // DEBUG("update " << robot_id);
    if (!ctrl.ignore)
    {
      // DEBUG("not ignore");
      if (!ctrl.active)
      {
        // DEBUG("not active");
        set(robot_id, true, 0.0, 0.0, 0.0);
      }
      else
      {
        // DEBUG("active");
        // if( robot_id == 1 ){
        //    DEBUG( "CTRL : " << ctrl );
        //}
        int kick = 0;
        if (ctrl.kick)
          kick = 1;
        else if (ctrl.chip_kick)
          kick = 2;

        if (ctrl.tare_odom)
        {
          set(robot_id, true, ctrl.fix_translation[0], ctrl.fix_translation[1], ctrl.fix_rotation.value(), kick,
              ctrl.kick_power, ctrl.spin, ctrl.charge, ctrl.tare_odom

          );
          // DEBUG("TARE ODOM : " << ctrl.spin);
        }
        else
        {
          //          DEBUG("NOT TARE ODOM : " << robot_id << " " << true << " " << ctrl.linear_velocity[0] << " "
          //                                   << ctrl.linear_velocity[1] << " " << ctrl.angular_velocity.value() << " "
          //                                   << kick
          //                                   << " kpow" << ctrl.kick_power << " spin: " << ctrl.spin << " charge " <<
          //                                   ctrl.charge
          //                                   << " tare" << ctrl.tare_odom);
          set(robot_id, true, ctrl.linear_velocity[0], ctrl.linear_velocity[1], ctrl.angular_velocity.value(), kick,
              ctrl.kick_power, ctrl.spin, ctrl.charge, ctrl.tare_odom);
        }
      }
    }
  }
}

void Commander::updateElectronicInformations()
{
  for (unsigned int id = 0; id < MAX_ROBOTS; id++)
  {
    real_->updateRobot(id, Data::get()->robots[Ally][id].electronics);
    // DEBUG("update elec for " << id << " " << (int)Data::get()->robots[Ally][id].electronics.xpos);
  }
}

void Commander::send()
{
  for (auto& cmd : commands_)
  {
    if (ai::Config::is_in_simulation)
    {
      grSim_Packet packet = convertToSimulationPacket(cmd);
      sim_->sendPacket(packet);
    }
    else
    {
      struct packet_master packet = convertToRobotPacket(cmd);
      real_->addRobotPacket(cmd.robot_id, packet);
    }
  }

  commands_.clear();

  if (!ai::Config::is_in_simulation)
    real_->send();
}

bool Commander::runTask()
{
  if (!ai::Config::is_in_simulation)
    updateElectronicInformations();

  if (Data::get()->referee.getCurrentStateName() == "HALT")
  {
    Data::get()->shared_data.final_control_for_robots[Data::get()->referee.teams_info->goalkeeper_number].control =
        Control::makeNull();
  }

  updateRobotsCommands();

  send();

  return true;
}

}  // namespace control
}  // namespace rhoban_ssl
