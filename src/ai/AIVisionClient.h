#pragma once

#include <VisionClient.h>
#include "GameState.h"

namespace RhobanSSL
{
class AIVisionClient : public VisionClient
{
public:
    typedef enum {
        Yellow, Blue
    } Team;

    AIVisionClient(Team myTeam, bool simulation = false);
    GameState &getGameState();

protected:
    virtual void packetReceived();

    Team myTeam;
    GameState gameState;

    void updateRobotInformation(SSL_DetectionRobot &robot, bool ally);
};
}
