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

protected:
    SimClient client;
};
}
