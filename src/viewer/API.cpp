#include <QThread>
#include <unistd.h>
#include "API.h"
#include <com/AICommanderReal.h>

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
    commander(commander),
    comThread(NULL)
{
    for (int id=0; id<MAX_ROBOTS; id++) {
        robots[id].id = id;
        robots[id].enabled = false;
    }

    comThread = new std::thread([this] {
        this->comThreadExec();
    });
}

API::~API()
{
    if (comThread) {
        comThread->join();
        delete comThread;
    }
}

void API::comThreadExec()
{
    // Communicating with robots
    while (true) {
        mutex.lock();
        bool hasRobot = false;
        for (int id=0; id<MAX_ROBOTS; id++) {
            auto &robot = robots[id];
            if (robot.enabled) {
                hasRobot = true;
                commander->set(id, true, robot.xSpeed, robot.ySpeed,
                    robot.thetaSpeed, robot.kick, robot.spin, robot.charge);
            }
        }
        if (hasRobot) {
            commander->flush();
        }
        mutex.unlock();
        QThread::msleep(10);
    }
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
            jsonRobot["present"] = robot.isOk();
            jsonRobot["team"] = ((team == RhobanSSL::Vision::Ally) ? ourColor() : opponentColor());
            auto pos = robot.movement[0];
            jsonRobot["x"] = pos.linear_position.x;
            jsonRobot["y"] = pos.linear_position.y;
            jsonRobot["orientation"] = pos.angular_position.value();

            auto &apiRobot = robots[robot.id];
            if (team == RhobanSSL::Vision::Ally) {
                if (!simulation) {
                    // XXX: This is not thread safe, to fix
                    RhobanSSL::Master *master = dynamic_cast<RhobanSSL::AICommanderReal*>(commander)->getMaster();
                    auto masterRobot = master->robots[robot.id];
                    jsonRobot["com"] = masterRobot.present && ((masterRobot.status.status) & STATUS_OK);
                    jsonRobot["voltage"] = masterRobot.status.voltage/10.0;
                    jsonRobot["capVoltage"] = masterRobot.status.cap_volt;
                }
                jsonRobot["team"] = ourColor();
                jsonRobot["enabled"] = apiRobot.enabled;
                jsonRobot["charge"] = apiRobot.charge;
            } else {
                jsonRobot["team"] = opponentColor();
                jsonRobot["enabled"] = false;
                jsonRobot["charge"] = false;
            }

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

QString API::fieldStatus()
{
    Json::Value json;

    RhobanSSL::Vision::VisionData vision;
    data >> vision;

    auto &field = vision.field;
    json["present"] = field.present;
    json["length"] = field.fieldLength;
    json["width"] = field.fieldWidth;
    json["goalWidth"] = field.goalWidth;
    json["goalDepth"] = field.goalDepth;
    json["boundaryWidth"] = field.boundaryWidth;

    return js(json);
}

void API::moveBall(double x, double y)
{
    mutex.lock();
    commander->moveBall(x, y);
    mutex.unlock();
}

void API::moveRobot(bool yellow, int id, double x, double y, double theta)
{
    mutex.lock();
    commander->moveRobot(yellow, id, x, y, theta, true);
    visionClient.setRobotPos(yellow ? RhobanSSL::AIVisionClient::Yellow : RhobanSSL::AIVisionClient::Blue,
        id, x, y, theta);
    mutex.unlock();
}

void API::enableRobot(int id, bool enabled)
{
    mutex.lock();
    if (id > 0 && id < MAX_ROBOTS) {
        robots[id].enabled = enabled;

        if (!robots[id].enabled) {
            robots[id].charge = false;
            robots[id].spin = false;
        }
    }
    mutex.unlock();
}

void API::robotCommand(int id,
    double xSpeed, double ySpeed, double thetaSpeed, int kick, bool spin, bool charge)
{
    mutex.lock();
    if (robots[id].enabled) {
        robots[id].xSpeed = xSpeed;
        robots[id].ySpeed = ySpeed;
        robots[id].thetaSpeed = thetaSpeed;
        robots[id].kick = kick;
        robots[id].spin = spin;
        robots[id].charge = charge;
    }
    mutex.unlock();
}

void API::robotCharge(int id, bool charge)
{
    mutex.lock();
    if (robots[id].enabled) {
        robots[id].charge = charge;
    }
    mutex.unlock();
}

void API::kick(int id, int kick)
{
    mutex.lock();
    if (robots[id].enabled) {
        robots[id].kick = kick;
    }
    mutex.unlock();
}

void API::emergencyStop()
{
    mutex.lock();
    for (int id=0; id<MAX_ROBOTS; id++) {
        robots[id].enabled = false;
    }

    for (int k=0; k<3; k++) {
        commander->stopAll();
        commander->flush();
    }
    mutex.unlock();
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

void API::scan()
{
    mutex.lock();
    // Sending a disable packet to all robots
    for (int n=0; n<3; n++) {
        for (int id=0; id<MAX_ROBOTS; id++) {
            commander->set(id, false, 0, 0, 0);
        }
        commander->flush();
    }

    // Enabling robots depending on their statuses
    for (int id=0; id<MAX_ROBOTS; id++) {
        if (simulation) {
            if (id < 7) {
                robots[id].enabled = true;
            }
        } else {
            // XXX: This is not thread safe, to fix
            RhobanSSL::Master *master = dynamic_cast<RhobanSSL::AICommanderReal*>(commander)->getMaster();
            auto masterRobot = master->robots[id];
            robots[id].enabled = masterRobot.present && ((masterRobot.status.status) & STATUS_OK);
        }
    }
    mutex.unlock();
}
