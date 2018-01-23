#include <iostream>
#include "AIVisionClient.h"
#include "debug.h"

using namespace Utils::Timing;

namespace RhobanSSL
{
    AIVisionClient::AIVisionClient(AIVisionClient::Team myTeam, bool simulation)
    : myTeam(myTeam), VisionClient(simulation)
    {
    }

    void AIVisionClient::packetReceived()
    {
        // Retrieving field dimensions
        auto geometry = data.geometry();
        if (geometry.has_field()) {
            gameState.field.present = true;
            gameState.field.fieldLength = geometry.field().field_length()/1000.0;
            gameState.field.fieldWidth = geometry.field().field_width()/1000.0;
            gameState.field.goalWidth = geometry.field().goal_width()/1000.0;
        }

        auto detection = data.detection();

        // Ball informations
        if (detection.balls().size()) {
            if (!gameState.ball.present || gameState.ball.age() > 1) {
                // If the ball is outdated (> 1s) or not present, taking the first
                // one in the frame
                double x = detection.balls().Get(0).x()/1000.0;
                double y = detection.balls().Get(0).y()/1000.0;
                gameState.ball.position = Point(x, y);
                gameState.ball.present = true;
            } else {
                // Else, we accept the ball which is the nearest from the previous one
                // we already had
                bool hasBall = false;
                Point bestBall;
                double nearest;

                for (auto ball : detection.balls()) {
                    double x = ball.x()/1000.0;
                    double y = ball.y()/1000.0;
                    Point pos(x, y);

                    double distance = pos.getDist(gameState.ball.position);
                    if (!hasBall || distance < nearest) {
                        nearest = distance;
                        bestBall = pos;
                    }
                }

                gameState.ball.position = bestBall;
            }

            gameState.ball.lastUpdate = TimeStamp::now();
        }

        // Robots informations
        for (auto robot : detection.robots_blue()) {
            updateRobotInformation(robot, myTeam == Blue);
        }
        for (auto robot : detection.robots_yellow()) {
            updateRobotInformation(robot, myTeam == Yellow);
        }


    }

    void AIVisionClient::updateRobotInformation(SSL_DetectionRobot &robotFrame, bool ally)
    {
        GameState::Team team = ally ? GameState::Team::Ally : GameState::Team::Opponent;
        GameState::Robot &robot = gameState.robots[team][robotFrame.robot_id()];

        robot.position = Point(robotFrame.x()/1000.0, robotFrame.y()/1000.0);

        if (robotFrame.has_orientation()) {
            robot.orientation = Angle(robotFrame.orientation());
        }

        robot.present = true;
        robot.lastUpdate = TimeStamp::now();
    }

    GameState &AIVisionClient::getGameState()
    {
        return gameState;
    }
}
