#include "AI.h"
#include <timing/TimeStamp.hpp>
#include <cmath>
#include <unistd.h>

using namespace Utils::Timing;

namespace RhobanSSL
{
    AI::AI(AIVisionClient *vision, AICommander *commander)
    : vision(vision), commander(commander)
    {
        running = true;
    }

    void AI::tick()
    {
        auto gameState = vision->getGameState();

        // Moving the robot 0 to the center of the field
        auto robot = gameState.robots[GameState::Ally][0];
        if (robot.isOk()) {
            double xSpeed = -robot.position.getX();
            double ySpeed = -robot.position.getY();
            double orientation = -robot.orientation.getSignedValue()*0.1;

            if (fabs(orientation) > 3) {
                xSpeed = ySpeed = 0;
            }
            if (xSpeed > 3) xSpeed = 3;
            if (xSpeed < -3) xSpeed = -3;

            if (ySpeed > 3) ySpeed = 3;
            if (ySpeed < -3) ySpeed = -3;

            if (orientation > 3) orientation = 3;
            if (orientation < -3) orientation = -3;

            commander->set(0, true, xSpeed, ySpeed, orientation);
            commander->flush();
        }
    }

    void AI::run()
    {
        double period = 1/100.0;    // 100 hz
        auto lastTick = TimeStamp::now();

        while (running) {
            auto now = TimeStamp::now();
            double elapsed = diffSec(lastTick, now);
            double toSleep = period - elapsed;
            if (toSleep > 0) {
                usleep(round(toSleep*1000000));
            }
            lastTick = TimeStamp::now();
            tick();
        }
    }

    void AI::stop()
    {
        running = false;
    }
}
