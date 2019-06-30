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

#pragma once

#include <stdint.h>
#include <Master.h>
#include <SimClient.h>
#include <execution_manager.h>

namespace rhoban_ssl
{
namespace control
{
class Commander : public Task
{
public:
  Commander();
  ~Commander();

  /**
   * Set the speed of the robot robot_id to the given speed
   */
  void set(uint8_t robot_id, bool enabled, double x_speed, double y_speed, double theta_speed, int kick = false,
           float kick_power = 0, bool spin = false, bool charge = false, bool tare_odom = false);

  /**
   * Stop all the robots
   */
  void stopAll();

  void moveBall(double x, double y, double vx, double vy);

  void moveRobot(bool ally, int id, double x, double y, double theta, bool turn_on);

private:
  Master* real_;
  SimClient* sim_;

  struct Command
  {
    uint8_t robot_id;
    bool enabled;
    double x_speed;
    double y_speed;
    double theta_speed;
    int kick;
    float kick_power;
    bool spin;
    bool charge;
    bool tare_odom;
  };

  std::vector<struct Command> commands_;

  grSim_Packet convertToSimulationPacket(const Command& cmd);

  struct packet_master convertToRobotPacket(const Command& cmd);

  void updateRobotsCommands();
  void updateElectronicInformations();
  void send();

  // Task interface
public:
  bool runTask();
};

}  // namespace control
}  // namespace rhoban_ssl
