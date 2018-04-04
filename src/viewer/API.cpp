#include "API.h"

API::API(bool simulation, bool yellow)
:
    simulation(simulation),
    yellow(yellow)
{
}

API::~API()
{
}

bool API::isSimulation()
{
    return simulation;
}

bool API::isYellow()
{
    return yellow;
}
