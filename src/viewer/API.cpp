#include "API.h"

// Helper, converts a json value to its string representation
static QString js(Json::Value &json)
{
    Json::FastWriter writer;

    return QString::fromStdString(writer.write(json));
}

API::API(bool simulation, RhobanSSL::AIVisionClient::Team team, RhobanSSL::AICommander *commander)
:
    simulation(simulation),
    team(team),
    visionClient(data, team, simulation),
    commander(commander)
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

QString API::visionStatus()
{
    Json::Value json;

    json["hasData"] = visionClient.hasData();
    json["packets"] = visionClient.getPackets();

    return js(json);
}

QString API::robotsStatus()
{
    Json::Value json(Json::arrayValue);

    RhobanSSL::Vision::VisionData vision;
    data >> vision;

    for (auto &entry : vision.robots) {
        auto &team = entry.first;
        for (auto &entry2 : entry.second) {
            Json::Value jsonRobot;
            auto &robot = entry2.second;

            jsonRobot["id"] = robot.id;
            jsonRobot["present"] = robot.present;
            jsonRobot["team"] = ((team == RhobanSSL::Vision::Ally) ? ourColor() : opponentColor());
            auto pos = robot.movement[0];
            jsonRobot["x"] = pos.linear_position.x;
            jsonRobot["y"] = pos.linear_position.y;
            jsonRobot["orientation"] = pos.angular_position.value();

            json.append(jsonRobot);
        }
    }

    return js(json);
}

QString API::ballStatus()
{
    Json::Value json;

    RhobanSSL::Vision::VisionData vision;
    data >> vision;

    auto pos = vision.ball.movement[0];
    json["x"] = pos.linear_position.x;
    json["y"] = pos.linear_position.y;

    return js(json);
}

void API::moveBall(double x, double y)
{
    commander->moveBall(x, y);
}

void API::moveRobot(bool yellow, int id, double x, double y, double theta)
{
    commander->moveRobot(yellow, id, x, y, theta, true);
}

void API::robotCommand(int id, bool enabled,
    double xSpeed, double ySpeed, double thetaSpeed, int kick, bool spin)
{
    commander->set(id, enabled, xSpeed, ySpeed, thetaSpeed, kick, spin);
    commander->flush();
}

void API::emergencyStop()
{
    for (int k=0; k<3; k++) {
        commander->stopAll();
        commander->flush();
    }
}

std::string API::ourColor()
{
    if (isYellow()) {
        return "yellow";
    } else {
        return "blue";
    }
}

std::string API::opponentColor()
{
    if (isYellow()) {
        return "blue";
    } else {
        return "yellow";
    }
}
