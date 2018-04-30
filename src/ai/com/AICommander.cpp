#include "AICommander.h"

namespace RhobanSSL
{
    AICommander::AICommander(bool yellow)
    : yellow(yellow)
    {
    }

    AICommander::~AICommander(){ }

    void AICommander::set(uint8_t robot_id, bool enabled,
        double xSpeed, double ySpeed, double thetaSpeed, int kick, bool spin, bool charge)
    {
        Command command;
        command.enabled = enabled;
        command.robot_id = robot_id;
        command.xSpeed = xSpeed;
        command.ySpeed = ySpeed;
        command.thetaSpeed = thetaSpeed;
        command.kick = kick;
        command.spin = spin;
        command.charge = charge;

        commands.push_back(command);
    }

    void AICommander::stopAll()
    {
        for (int k=0; k<8; k++) {
            set(k, false, 0, 0, 0);
        }
    }
}
