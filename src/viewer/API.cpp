#include "API.h"

API::API(bool simulation, RhobanSSL::AIVisionClient::Team team)
:
    simulation(simulation),
    team(team),
    visionClient(data, team, simulation)
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
    return team == RhobanSSL::AIVisionClient::Yellow;
}

bool API::hasVisionData()
{
    return visionClient.hasData();
}

unsigned int API::visionPackets()
{
    return visionClient.packets;
}
