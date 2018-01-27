#include <iostream>
#include "AIVisionClient.h"
#include <tools/debug.h>

using namespace Utils::Timing;

namespace RhobanSSL
{
    AIVisionClient::AIVisionClient(
        Data& ai_data, AIVisionClient::Team myTeam, bool simulation
    ): ai_data(ai_data), myTeam(myTeam), VisionClient(simulation)
    {
    }

    void AIVisionClient::packetReceived()
    {
        // Retrieving field dimensions
        auto geometry = data.geometry();
        if (geometry.has_field()) {
            visionData.field.present = true;
            visionData.field.fieldLength = geometry.field().field_length()/1000.0;
            visionData.field.fieldWidth = geometry.field().field_width()/1000.0;
            visionData.field.goalWidth = geometry.field().goal_width()/1000.0;
        }

        auto detection = data.detection();

        // Ball informations
        if (detection.balls().size()) {
            if (!visionData.ball.present || visionData.ball.age() > 1) {
                // If the ball is outdated (> 1s) or not present, taking the first
                // one in the frame
                double x = detection.balls().Get(0).x()/1000.0;
                double y = detection.balls().Get(0).y()/1000.0;
                visionData.ball.update(detection.t_capture(), Point(x,y));
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

                    double distance = pos.getDist( 
                        visionData.ball.movement[0].linear_position
                    );
                    if (!hasBall || distance < nearest) {
                        nearest = distance;
                        bestBall = pos;
                    }
                }
                visionData.ball.update(detection.t_capture(), bestBall);
            }
        }

        // Robots informations
        for (auto robot : detection.robots_blue()) {
            updateRobotInformation(detection, robot, myTeam == Blue);
        }
        for (auto robot : detection.robots_yellow()) {
            updateRobotInformation(detection, robot, myTeam == Yellow);
        }

        ai_data << visionData;
    }

    void AIVisionClient::updateRobotInformation(
        SSL_DetectionFrame &detection, 
        SSL_DetectionRobot &robotFrame, bool ally
    ){
        Vision::Team team = ally ? Vision::Team::Ally : Vision::Team::Opponent;
        Vision::Robot &robot = visionData.robots[team][robotFrame.robot_id()];

        Point position = Point(robotFrame.x()/1000.0, robotFrame.y()/1000.0);

        if (robotFrame.has_orientation()) {
            Angle orientation(robotFrame.orientation());
            robot.update( detection.t_capture(), position, orientation );
        }else{
            robot.update( detection.t_capture(), position );
        }
    }

}
