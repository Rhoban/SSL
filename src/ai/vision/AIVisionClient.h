#pragma once

#include <VisionClient.h>
#include "VisionData.h"
#include <Data.h>
#include <AiData.h>

namespace RhobanSSL
{
class AIVisionClient : public VisionClient
{
public:

    AIVisionClient(Data& shared_data, Ai::Team myTeam, bool simulation = false);

    void setRobotPos(Ai::Team team, int id, double x, double y, double orientation);

protected:
    virtual void packetReceived();

    Data & shared_data;

    void updateRobotInformation(
        SSL_DetectionFrame &detection,
        SSL_DetectionRobot &robot, bool ally
    );
    
    private:
    Vision::VisionData visionData;
    Ai::Team myTeam;
};
}
