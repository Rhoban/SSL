#pragma once

#include <VisionClient.h>
#include "VisionData.h"
#include <Data.h>

namespace RhobanSSL
{
class AIVisionClient : public VisionClient
{
public:
    typedef enum {
        Yellow, Blue
    } Team;

    AIVisionClient(Data& data, Team myTeam, bool simulation = false);

protected:
    virtual void packetReceived();

    Data & ai_data;
    Team myTeam;
    Vision::VisionData visionData;

    void updateRobotInformation(
        SSL_DetectionFrame &detection,
        SSL_DetectionRobot &robot, bool ally
    );
};
}
