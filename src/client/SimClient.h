#pragma once

#include <string>
#include <rhoban_utils/sockets/udp_broadcast.h>
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"

namespace rhoban_ssl
{
/**
 * A client that can communicate with the simulator to send orders or to
 * control robot
 */
class SimClient
{
public:
  SimClient();

  SimClient(std::string port);
  /**
   * Move the ball the the desired x,y position with desired vx,vy speed
   */
  void moveBall(double x, double y, double vx = 0, double vy = 0);

  /**
   * Moves a robot to a position
   *
   * @param yellow Is the robot yellow team?
   * @param id     The robot id
   * @param x      Robot position (Y, [m])
   * @param y      Robot position (Y, [m])
   * @param theta  Robot orientation [deg]
   * @param turnon Is the robot on ?
   */
  void moveRobot(bool yellow, int id, double x, double y, double theta, bool turnon);

  /**
   * Controls a robot
   *
   * @param yellow Is the robot yellow ?
   * @param id     The robot id
   * @param x      Robot position (X, [m])
   * @param y      Robot position (Y, [m])
   * @param theta  Robot orientation [deg]
   * @param kickX  Kick X [m/s]
   * @param kickZ  Kick Y [m/s]
   * @param spin   Is the dribbler spinning ?
   */
  void send(
      // Robot id
      bool yellow, int id,
      // Robot speed
      double x, double y, double theta,
      // Robot kick
      double kickX, double kickZ, bool spin);

  void sendPacket(grSim_Packet& packet);

protected:
  rhoban_utils::UDPBroadcast broadcast;
};
}  // namespace rhoban_ssl
