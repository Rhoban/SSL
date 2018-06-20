#include "SimClient.h"
#include "client_config.h"
#include <string>

namespace RhobanSSL
{
SimClient::SimClient()
  : broadcast(-1, SSL_SIM_PORT)
{
}

SimClient::SimClient(std::string port)
  : broadcast(-1, stoi(port))
{
}

void SimClient::moveBall(double x, double y, double vx, double vy)
{
  grSim_Packet packet;

  auto replacement = packet.mutable_replacement();
  auto ball = replacement->mutable_ball();

  ball->set_x(x);
  ball->set_y(y);
  ball->set_vx(vx);
  ball->set_vy(vy);

  sendPacket(packet);
}

void SimClient::moveRobot(bool yellow, int id,
                          double x, double y, double theta,
                          bool turnon)
{
  grSim_Packet packet;

  auto replacement = packet.mutable_replacement();
  auto robot = replacement->add_robots();

  robot->set_yellowteam(yellow);
  robot->set_id(id);
  robot->set_x(x);
  robot->set_y(y);
  robot->set_dir(theta);
  robot->set_turnon(turnon);

  sendPacket(packet);
}

void SimClient::send(bool yellow, int id,
                     double x, double y, double theta,
                     double kickX, double kickZ, bool spin)
{
  // Building packet
  grSim_Packet packet;
  packet.mutable_commands()->set_isteamyellow(yellow);
  packet.mutable_commands()->set_timestamp(0.0);
  grSim_Robot_Command *command = packet.mutable_commands()->add_robot_commands();

  // Appending data
  command->set_id(id);
  command->set_wheelsspeed(false);
  command->set_veltangent(x);
  command->set_velnormal(y);
  command->set_velangular(theta);
  command->set_kickspeedx(kickX);
  command->set_kickspeedz(kickZ);
  command->set_spinner(spin);

  sendPacket(packet);
}

double SimClient::sendPacket(grSim_Packet &packet)
{
  // Broadcasting the packet
  size_t len = packet.ByteSize();
  unsigned char buffer[len];
  packet.SerializeToArray(buffer, len);
  broadcast.broadcastMessage(buffer, len);
  return rhoban_utils::TimeStamp::now().getTimeSec();
}
}
