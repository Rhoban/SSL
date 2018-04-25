#include <QThread>
#include <unistd.h>
#include "API.h"
#include <annotations/Annotations.h>
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
    comThread(NULL),
    joystick(NULL),
    joystickRobot(0)
{
    for (int id=0; id<MAX_ROBOTS; id++) {
        robots[id].id = id;
        robots[id].enabled = false;
        robots[id].xSpeed = 0;
        robots[id].ySpeed = 0;
        robots[id].thetaSpeed = 0;
        robots[id].charge = false;
        robots[id].spin = false;
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
                    jsonRobot["com"] = masterRobot.isOk();
                    jsonRobot["voltage"] = masterRobot.status.voltage/8.0;
                    jsonRobot["capVoltage"] = masterRobot.status.cap_volt;
                    jsonRobot["driversOk"] = !(masterRobot.status.status & STATUS_DRIVER_ERR);
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
    if (id >= 0 && id < MAX_ROBOTS) {
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
    stopJoystick();

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
            if (id <= 7) {
                robots[id].enabled = true;
            }
        } else {
            // XXX: This is not thread safe, to fix
            RhobanSSL::Master *master = dynamic_cast<RhobanSSL::AICommanderReal*>(commander)->getMaster();
            auto masterRobot = master->robots[id];
            robots[id].enabled = masterRobot.isOk();
        }
    }
    mutex.unlock();
}

void API::joystickTrheadExec()
{
    static bool fast = false;
    RhobanSSL::Joystick::JoystickEvent event;
    if (joystick != NULL) {
        joystick->open();
    }
    while ((joystickRobot >= 0) && (joystick != NULL)) {
        while (joystick->getEvent(&event)) {
            mutex.lock();
            if (event.type == JS_EVENT_AXIS) {
                if (event.number == 0) {
                    robots[joystickRobot].ySpeed = -(fast ? 2.0 : 0.5)*event.getValue();
                }
                if (event.number == 1) {
                    robots[joystickRobot].xSpeed = -(fast ? 2.0 : 0.5)*event.getValue();
                }
                if (event.number == 3) {
                    robots[joystickRobot].thetaSpeed = -1.5*event.getValue();
                }

                std::cout << "[JOYSTICK] Axis #" << (int)event.number << ": " << event.getValue() << std::endl;;
            }
            if (event.type == JS_EVENT_BUTTON) {
                if (event.number == 4) {
                    robots[joystickRobot].spin = event.isPressed();
                }
                if (event.number == 5) {
                    robots[joystickRobot].kick = event.isPressed() ? 1 : 0;
                }
                if (event.number == 7) {
                    robots[joystickRobot].kick = event.isPressed() ? 2 : 0;
                }
                if (event.number == 6) {
                    fast = event.getValue();
                }

                std::cout << "[JOYSTICK] Button #" << (int)event.number << ": " << (int)event.isPressed() << std::endl;;
            }
            mutex.unlock();
        }

        QThread::msleep(10);
    }

    if (joystick != NULL) {
        delete joystick;
        joystick->close();
        joystick = NULL;
    }
}

void API::openJoystick(int robot, QString device)
{
    if (joystick == NULL) {
        joystickRobot = robot;
        joystick = new RhobanSSL::Joystick(device.toStdString());

        joystickThread = new std::thread([this] {
            this->joystickTrheadExec();
        });
    }
}

void API::stopJoystick()
{
    joystickRobot = -1;
}

void API::tweakPid(int id)
{
    if (!simulation) {
        mutex.lock();
        RhobanSSL::Master *master = dynamic_cast<RhobanSSL::AICommanderReal*>(commander)->getMaster();

        struct packet_params params;
        params.kp = 100;
        params.ki = 1;
        params.kd = 0;
        master->addParamPacket(id, params);
        master->send();
        master->send();

        mutex.unlock();
    }
}

QString API::availableJoysticks()
{
    Json::Value json(Json::arrayValue);

    for (auto joystick : RhobanSSL::Joystick::getAvailablePads()) {
        json.append(joystick);
    }

    return js(json);
}

QString API::getAnnotations()
{
    static double d = 0;
    d += 0.01;
    RhobanSSLAnnotation::Annotations annotations;


    // XXX: Annotations examples
    annotations.addCircle(3, 3, 1, "cyan");
    annotations.addArrow(0, 0, cos(d), sin(d)*2, "magenta", true);
    annotations.addCross(-3, 0, "red");
    // XXX: Texte
    // XXX: Segment


    return QString::fromStdString(annotations.toJsonString());
}

QString API::getStrategies()
{
    Json::Value json(Json::arrayValue);

    {
        Json::Value goal;
        Json::Value params;
        params["robot"] = "1";
        params["goalDist"] = "0.5";
        goal["name"] = "goal";
        goal["params"] = params;
        json.append(goal);
    }

    {
        Json::Value wall;
        Json::Value params;
        params["robot1"] = "2";
        params["robot2"] = "3";
        wall["name"] = "wall";
        wall["params"] = params;
        json.append(wall);
    }

    return js(json);
}

QString API::getManagers()
{
    Json::Value json(Json::arrayValue);

    json.append("Manual");
    json.append("RoboCup");

    return js(json);
}

void API::managerStop()
{
}

void API::managerPlay()
{
}

void API::managerPause()
{
}
