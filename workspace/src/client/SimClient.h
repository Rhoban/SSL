#pragma once

#include <string>
#include <sockets/UDPBroadcast.hpp>
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"

namespace RhobanSSL
{
class SimClient
{
public:
    SimClient();

    void moveBall(double x, double y, double vx, double vy);

    void moveRobot(bool yellow, int id,
        double x, double y, double theta,
        bool turnon);

    void send(
        // Robot id
        bool yellow, int id,
        // Robot speed
        double x, double y, double theta,
        // Robot kick
        double kickX, double kickZ, bool spin
    );

protected:
    Rhoban::UDPBroadcast broadcast;

    void sendPacket(grSim_Packet &packet);
};
}
