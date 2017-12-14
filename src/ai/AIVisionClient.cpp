#include <iostream>
#include "AIVisionClient.h"

namespace RhobanSSL
{
AIVisionClient::AIVisionClient(AIVisionClient::Team myTeam)
: myTeam(myTeam), VisionClient()
{
}

void AIVisionClient::packetReceived()
{
    std::cout << data.detection().frame_number() << std::endl;
}

GameState &AIVisionClient::getGameState()
{
    return gameState;
}
}
