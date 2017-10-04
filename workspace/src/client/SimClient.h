#pragma once

#include <string>
#include <sockets/UDPBroadcast.hpp>

namespace RhobanSSL
{
class SimClient
{
public:
    SimClient();

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
};
}
