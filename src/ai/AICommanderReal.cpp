#include "AICommanderReal.h"

namespace RhobanSSL
{
    AICommanderReal::AICommanderReal(bool yellow)
    : AICommander(yellow), master("/dev/ttyACM0", 1000000)
    {
        for (int k=0; k<6; k++) {
            master.robots[k].kickPower = 10000;
        }
    }

    void AICommanderReal::flush()
    {
        for (auto &command : commands) {
            auto wheels = kinematic.compute(command.xSpeed, command.ySpeed, command.thetaSpeed);
            if (command.enabled) {
                master.robots[command.robot_id].actions = ACTION_ON;
            } else {
                master.robots[command.robot_id].actions = 0;
            }
            master.robots[command.robot_id].wheel1 = wheels.frontLeft;
            master.robots[command.robot_id].wheel2 = wheels.backLeft;
            master.robots[command.robot_id].wheel3 = wheels.backRight;
            master.robots[command.robot_id].wheel4 = wheels.frontRight;
        }

        master.send();
        commands.clear();
    }
}
