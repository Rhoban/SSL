#include "AICommanderSimulation.h"
#include <geometry/Angle.hpp>

namespace RhobanSSL
{
    AICommanderSimulation::AICommanderSimulation(bool yellow)
    : AICommander(yellow), client()
    {
    }

    void AICommanderSimulation::flush()
    {
        grSim_Packet packet;
        packet.mutable_commands()->set_isteamyellow(yellow);
        packet.mutable_commands()->set_timestamp(0.0);

        for (auto &command : commands) {
            double factor = command.enabled ? 1 : 0;

            grSim_Robot_Command *simCommand = packet.mutable_commands()->add_robot_commands();

            // Appending data
            simCommand->set_id(command.robot_id);
            simCommand->set_wheelsspeed(false);
            simCommand->set_veltangent(command.xSpeed*factor);
            simCommand->set_velnormal(command.ySpeed*factor);
            simCommand->set_velangular( command.thetaSpeed*factor );
            simCommand->set_kickspeedx(0);
            simCommand->set_kickspeedz(0);
            simCommand->set_spinner(false);
        }

        client.sendPacket(packet);
        commands.clear();
    }
}
