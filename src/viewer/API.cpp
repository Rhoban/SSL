#include "API.h"

// Helper, converts a json value to its string representation
static QString js(Json::Value &json)
{
    Json::FastWriter writer;

    return QString::fromStdString(writer.write(json));
}

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
