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
        int kick;
        bool spin;
    };
    AICommander(bool yellow);

    /**
     * Set the speed of the robot robot_id to the given speed
     */
    void set(uint8_t robot_id, bool enabled,
        double xSpeed, double ySpeed, double thetaSpeed, int kick=false, bool spin=false);

    /**
     * Stop all the robots
     */
    void stopAll();

    /**
     * Call this method to flush the commands
     */
    virtual void flush()=0;
    virtual void kick(){};

    /**
     * Sets the ball position
     */
    virtual void moveBall(double x, double y, double vx=0, double vy=0){};

    /**
     * Moves a robot o a target position
     */
    virtual void moveRobot(bool yellow, int id, double x, double y,
        double theta, bool turnon){};

protected:
    std::vector<struct Command> commands;

    bool yellow;
};
}
