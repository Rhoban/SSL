#include "AICommanderReal.h"

namespace RhobanSSL
{
    AICommanderReal::AICommanderReal(bool yellow)
    : AICommander(yellow), master("/dev/ttyACM0", 1000000), kicking(false)
    {
    }

    void AICommanderReal::kick(){
        kicking = true;
    }

    void AICommanderReal::flush()
    {
        for (auto &command : commands) {
            struct packet_master packet;
            if (command.enabled) {
                packet.actions = ACTION_ON |ACTION_CHARGE;
                if( kicking ){
                    packet.actions |= ACTION_KICK1;
                }
            } else {
                packet.actions = 0;
            }
            packet.kickPower = 150;
            packet.x_speed = command.xSpeed;
            packet.y_speed = command.ySpeed;
            packet.t_speed = command.thetaSpeed;

            master.addRobotPacket(command.robot_id, packet);
        }

        master.send();
        commands.clear();
    }
}
