#include "SimClient.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "client_config.h"

namespace RhobanSSL
{
SimClient::SimClient()
: broadcast(-1, SSL_SIM_PORT)
{
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

    // Broadcasting the packet
    size_t len = packet.ByteSize();
    unsigned char buffer[len];
    packet.SerializeToArray(buffer, len);
    broadcast.broadcastMessage(buffer, len);
}
}
