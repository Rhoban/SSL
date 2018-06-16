/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 TO COMPLETE -> Gregwar

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

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
        float kickPower;
        bool spin;
        bool charge;
    };
    AICommander(bool yellow);

    void set_yellow(bool value);

    /**
     * Set the speed of the robot robot_id to the given speed
     */
    void set(uint8_t robot_id, bool enabled,
        double xSpeed, double ySpeed, double thetaSpeed, int kick=false, int kickPower=0, bool spin=false, bool charge=false);

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

    virtual ~AICommander();

protected:
    std::vector<struct Command> commands;

    bool yellow;
};
}
