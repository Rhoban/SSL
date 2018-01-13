#pragma once

#include <stdint.h>
#include <Master.h>
#include <SimClient.h>

/**
 * Generic interface for commanding robots, whatever it is in simulator or
 * not for instance
 */
namespace RhobanSSL
{
class AICommander
{
public:
    struct Command {
        uint8_t robot_id;
        bool enabled;
        double xSpeed;
        double ySpeed;
        double thetaSpeed;
    };
    AICommander(bool yellow);

    /**
     * Set the speed of the robot robot_id to the given speed
     */
    void set(uint8_t robot_id, bool enabled,
        double xSpeed, double ySpeed, double thetaSpeed);

    /**
     * Stop all the robots
     */
    void stopAll();

    /**
     * Call this method to flush the commands
     */
    virtual void flush()=0;

protected:
    std::vector<struct Command> commands;

    bool yellow;
};
}
