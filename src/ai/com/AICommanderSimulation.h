#pragma once

#include <stdint.h>
#include <SimClient.h>
#include "AICommander.h"

namespace RhobanSSL
{
class AICommanderSimulation : public AICommander
{
public:
    AICommanderSimulation(bool yellow);

    virtual void flush();

    virtual void moveBall(double x, double y, double vx=0, double vy=0);

    virtual void moveRobot(bool yellow, int id, double x, double y,
        double theta, bool turnon);

    virtual ~AICommanderSimulation();

protected:
    SimClient client;
};
}
