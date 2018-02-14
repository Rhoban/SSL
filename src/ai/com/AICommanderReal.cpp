#include "AICommanderReal.h"

namespace RhobanSSL
{
    AICommanderReal::AICommanderReal(bool yellow)
    : AICommander(yellow), master("/dev/ttyACM0", 1000000), kicking(false)
    {
        for (int k=0; k<6; k++) {
            master.robots[k].kickPower = 1800;
        }
    }

    void AICommanderReal::kick(){
        kicking = true;
    }

    void AICommanderReal::flush()
    {
        for (auto &command : commands) {
            if (command.enabled) {
                master.robots[command.robot_id].actions = ACTION_ON |ACTION_CHARGE;
                if( kicking ){
                    master.robots[command.robot_id].actions |= ACTION_KICK1;
                }
            } else {
                master.robots[command.robot_id].actions = 0;
            }
            master.robots[command.robot_id].x_speed = command.xSpeed;
            master.robots[command.robot_id].y_speed = command.ySpeed;
            master.robots[command.robot_id].t_speed = command.thetaSpeed;
        }

        master.send();
        commands.clear();
    }
}
