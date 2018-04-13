#include "AICommanderReal.h"

namespace RhobanSSL
{
    AICommanderReal::AICommanderReal(bool yellow)
    : AICommander(yellow), master("/dev/ttyACM0", 1000000), kicking(false)
    {
    }

    void AICommanderReal::kick(){
        kicking = true;
        // XXX Should not be used anymore
    }

    void AICommanderReal::flush()
    {
        // Transferring abstract commands to the master implementation
        for (auto &command : commands) {
            struct packet_master packet;
            if (command.enabled) {
                packet.actions = ACTION_ON;

                if (command.charge) {
                    packet.actions |= ACTION_CHARGE;
                }

                if (command.kick == 1) {
                    packet.actions |= ACTION_KICK1;
                }

                if (command.kick == 2) {
                    packet.actions |= ACTION_KICK2;
                }
            } else {
                packet.actions = 0;
            }
            packet.kickPower = 200; // XXX: This should be a parameter

            packet.x_speed = command.xSpeed*1000;
            packet.y_speed = command.ySpeed*1000;
            packet.t_speed = command.thetaSpeed*1000;

            master.addRobotPacket(command.robot_id, packet);
        }

        master.send();
        commands.clear();
    }

    Master *AICommanderReal::getMaster()
    {
        return &master;
    }
}
