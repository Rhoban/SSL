#include "AICommanderSimulation.h"
#include <rhoban_utils/angle.h>

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

            double kickX = 0;
            double kickY = 0;

            // XXX: These values should be realistic depending on what we do on the real robot!
            if (command.enabled) {
                if (command.kick == 1) {
                    kickX = 4;
                    kickY = 0;
                } else if (command.kick == 2) {
                    kickX = 2;
                    kickY = 2;
                }
            }

            // Appending data
            simCommand->set_id(command.robot_id);
            simCommand->set_wheelsspeed(false);
            simCommand->set_veltangent(command.xSpeed*factor);
            simCommand->set_velnormal(command.ySpeed*factor);
            simCommand->set_velangular( command.thetaSpeed*factor );
            simCommand->set_kickspeedx(kickX);
            simCommand->set_kickspeedz(kickY);
            simCommand->set_spinner(command.enabled ? command.spin : false);
        }

        client.sendPacket(packet);
        commands.clear();
    }

    void AICommanderSimulation::moveBall(double x, double y, double vx, double vy)
    {
        client.moveBall(x, y, vx, vy);
    }

    void AICommanderSimulation::moveRobot(bool yellow, int id, double x, double y,
        double theta, bool turnon)
    {
        client.moveRobot(yellow, id, x, y, theta*180/M_PI, turnon);
    }

    AICommanderSimulation::~AICommanderSimulation(){
    }

}
