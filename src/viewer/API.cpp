#include <QThread>
#include <unistd.h>
#include "API.h"
#include <robot_behavior/robot_behavior.h>
#include <annotations/Annotations.h>
#include <manager/Manual.h>
#include <com/AICommanderReal.h>
#include <manager/factory.h>

using namespace RhobanSSL;

#define NB_ROBOT_ELEC 8 //HACK !! TODO ! Electronic doen't support that number of robots

// Helper, converts a json value to its string representation
static QString js(Json::Value &json)
{
    Json::FastWriter writer;

    return QString::fromStdString(writer.write(json));
}

API::API(
  std::string teamName, bool simulation, RhobanSSL::Ai::Team team,
  RhobanSSL::AICommander *commander, const std::string & config_path,
  Vision::Part_of_the_field part_of_the_field_used, std::string addr, std::string port, std::string sim_port
  )
  :
  simulation(simulation),
  teamName(teamName),
  data(team),
  team(team),
  visionClient(data, team, simulation, addr, port, sim_port,part_of_the_field_used),
  commander(commander),
  joystick(NULL),
  joystickRobot(0)
{
    // Be sure that the final controls are initalized to no-speed
    RhobanSSL::Shared_data shared;
    data >> shared;
    for (int id=0; id<NB_ROBOT_ELEC; id++) {
        RhobanSSL::Control &control = shared.final_control_for_robots[id].control;

        control.ignore = true;
        control.charge = false;
        control.active = false;
        control.velocity_translation = Vector2d(0, 0);
        control.velocity_rotation = ContinuousAngle(0);
        control.spin = false;
        shared.final_control_for_robots[id].is_manually_controled_by_viewer = true;
    }
    data << shared;

    // Instanciating AI
    ai = new RhobanSSL::AI(
        Manager::names::match,
        teamName,
        team,
        data,
        commander, config_path, simulation
    );

    /*
    comThread = new std::thread([this] {
        this->comThreadExec();
    });
    */

    aiThread = new std::thread([this] {
        this->aiThreadExec();
    });
}

API::~API()
{
    if (aiThread) {
        ai->stop();
        aiThread->join();
        delete aiThread;
    }

    if (ai) {
        delete ai;
    }
}

/*
void API::comThreadExec()
{
    // Communicating with robots
    while (true) {
        mutex.lock();
        bool hasRobot = false;
        for (int id=0; id<NB_ROBOT_ELEC; id++) {
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
*/

void API::aiThreadExec()
{
    ai->run();
}

bool API::isSimulation()
{
    return simulation;
}

bool API::isYellow()
{
    Data_from_ai data_from_ai;
    data >> data_from_ai;
    return data_from_ai.team_color == Ai::Team::Yellow;
}

QString API::visionStatus()
{
    Json::Value json;

    json["hasData"] = visionClient.hasData();
    json["packets"] = visionClient.getPackets();

    return js(json);
}

QString API::refereeStatus()
{
    Json::Value json;

    auto &referee = ai->getReferee();
    auto &refereeClient = referee.getRefereeClient();
    SSL_Referee data = refereeClient.getData();
    json["our_color"] = ourColor();
    json["hasData"] = refereeClient.hasData();
    json["packets"] = refereeClient.getPackets();
    json["stage"] = SSL_Referee_Stage_Name(data.stage());
    json["command"] = SSL_Referee_Command_Name(data.command());
    json["time_left"] = data.stage_time_left();
    json["blue_positive"] = data.blueteamonpositivehalf();
    json["yellow_name"] = data.yellow().name();
    json["blue_name"] = data.blue().name();

    return js(json);
}

QString API::robotsStatus()
{
    Json::Value json(Json::arrayValue);

    RhobanSSL::Vision::VisionData vision;
    data >> vision;

    RhobanSSL::Shared_data shared;
    data >> shared;

    for (auto &entry : vision.robots) {
        auto &team = entry.first;
        for (auto &entry2 : entry.second) {
            Json::Value jsonRobot;
            auto &robot = entry2.second;

            jsonRobot["id"] = robot.id;
            jsonRobot["present"] = robot.isOk();
            jsonRobot["team"] = ((team == RhobanSSL::Vision::Ally) ? ourColor() : opponentColor());
            auto movement = robot.movement;
            jsonRobot["x"] = movement.linear_position().getX();
            jsonRobot["y"] = movement.linear_position().getY();
            jsonRobot["orientation"] = movement.angular_position().value();

            if (team == RhobanSSL::Vision::Ally) {
                auto final_control = shared.final_control_for_robots[robot.id];
                Control control = final_control.control;

                if (!simulation) {
                    // XXX: This should be read from the AI
                    RhobanSSL::Master *master = dynamic_cast<RhobanSSL::AICommanderReal*>(commander)->getMaster();
                    auto masterRobot = master->robots[robot.id];
                    jsonRobot["com"] = masterRobot.isOk();
                    jsonRobot["voltage"] = masterRobot.status.voltage/8.0;
                    jsonRobot["capVoltage"] = masterRobot.status.cap_volt;
                    jsonRobot["driversOk"] = !(masterRobot.status.status & STATUS_DRIVER_ERR);
                    
                    jsonRobot["ir"] = (masterRobot.status.status & STATUS_IR) ? true : false;
                    
                    jsonRobot["x_odom"] = (float)masterRobot.status.xpos/1000;
                    jsonRobot["y_odom"] = (float)masterRobot.status.ypos/1000;
                    jsonRobot["t_odom"] = (float)(masterRobot.status.ang);
                } else {
                    jsonRobot["ir"] = false;
                }
                jsonRobot["team"] = ourColor();
                jsonRobot["enabled"] = !control.ignore;
                jsonRobot["active"] = control.active;
                jsonRobot["manual"] = final_control.is_manually_controled_by_viewer;
                jsonRobot["charge"] = control.charge;
                jsonRobot["spin"] = control.spin;
                jsonRobot["tareOdom"] = control.tareOdom;
            } else {
                jsonRobot["team"] = opponentColor();
                jsonRobot["enabled"] = false;
                jsonRobot["charge"] = false;
                jsonRobot["ir"] = false;
                jsonRobot["spin"] = false;
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
    json["penaltyAreaWidth"] = field.penaltyAreaWidth;
    json["penaltyAreaDepth"] = field.penaltyAreaDepth;
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
    visionClient.setRobotPos(yellow ? Ai::Team::Yellow : Ai::Team::Blue,
        id, x, y, theta);
    mutex.unlock();
}

void API::enableRobot(int id, bool enabled)
{
    mutex.lock();
    if (id >= 0 && id < NB_ROBOT_ELEC) {
        RhobanSSL::Shared_data shared;
        data >> shared;
        RhobanSSL::Control &control = shared.final_control_for_robots[id].control;

        control.ignore = !enabled;

        if (control.ignore) {
            control.charge = false;
            control.spin = false;
        }

        data << shared;
    }
    mutex.unlock();
}

void API::manualControl(int id, bool manual)
{
    mutex.lock();
    if (id >= 0 && id < NB_ROBOT_ELEC) {
        RhobanSSL::Shared_data shared;
        data >> shared;
        auto &final_control = shared.final_control_for_robots[id];

        final_control.is_manually_controled_by_viewer = manual;
        data << shared;
    }
    mutex.unlock();
}

void API::activeRobot(int id, bool active)
{
    mutex.lock();
    if (id >= 0 && id < NB_ROBOT_ELEC) {
        RhobanSSL::Shared_data shared;
        data >> shared;
        RhobanSSL::Control &control = shared.final_control_for_robots[id].control;

        control.active = active;
        data << shared;
    }
    mutex.unlock();
}

void API::robotCommand(int id,
    double xSpeed, double ySpeed, double thetaSpeed)
{
    mutex.lock();

    RhobanSSL::Shared_data shared;
    data >> shared;
    RhobanSSL::Control &control = shared.final_control_for_robots[id].control;

    if (!control.ignore) {
        control.velocity_translation = Vector2d(xSpeed, ySpeed);
        control.velocity_rotation = ContinuousAngle(thetaSpeed);
    }

    data << shared;
    mutex.unlock();
}

void API::robotCharge(int id, bool charge)
{
    mutex.lock();
    RhobanSSL::Shared_data shared;
    data >> shared;
    RhobanSSL::Control &control = shared.final_control_for_robots[id].control;
    if (!control.ignore) {
        control.charge = charge;
    }
    data << shared;
    mutex.unlock();
}

void API::kick(int id, int kick, float power)
{
    mutex.lock();
    RhobanSSL::Shared_data shared;
    data >> shared;
    RhobanSSL::Control &control = shared.final_control_for_robots[id].control;

    if (!control.ignore) {
        control.kick = false;
        control.chipKick = false;
        control.kickPower = power;

        if (kick == 1) {
            control.kick = true;
        }
        if (kick == 2) {
            control.chipKick = true;
        }
    }
    data << shared;
    mutex.unlock();
}

void API::setSpin(int id, bool spin)
{
    mutex.lock();
    RhobanSSL::Shared_data shared;
    data >> shared;
    RhobanSSL::Control &control = shared.final_control_for_robots[id].control;

    if (!control.ignore) {
        control.spin = spin;
    }
    data << shared;
    mutex.unlock();
}

void API::tareOdom(int id, bool tare, double xFix, double yFix, double tFix)
{
    mutex.lock();
    RhobanSSL::Shared_data shared;
    data >> shared;
    RhobanSSL::Control &control = shared.final_control_for_robots[id].control;

    if (!control.ignore) {
        control.tareOdom = tare;
        
        //printf("MIAMMIAM\n\r");
        control.fix_translation = Vector2d(xFix, yFix);
        control.fix_rotation = ContinuousAngle(tFix);
        //printf("%f %f %f", xFix, yFix, tFix);
    }
    data << shared;
    mutex.unlock();
}

void API::emergencyStop()
{
    stopJoystick();

    mutex.lock();

    RhobanSSL::Shared_data shared;
    data >> shared;

    for (int id=0; id<NB_ROBOT_ELEC; id++) {
        auto &final_control = shared.final_control_for_robots[id];
        final_control.is_manually_controled_by_viewer = true;
        final_control.control.ignore = true;
        final_control.control.active = false;
    }

    data << shared;

    // XXX: Is this good?
    for (int k=0; k<3; k++) {
        commander->stopAll();
        commander->flush();
    }
    mutex.unlock();
}

std::string API::ourColor()
{
    Data_from_ai data_from_ai;
    data >> data_from_ai;

    return data_from_ai.team_color == Ai::Team::Yellow ? "yellow" : "blue";
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

    // Enabling robots depending on their statuses
    RhobanSSL::Shared_data shared;
    RhobanSSL::Shared_data shared_tmp_manual;
    data >> shared;
    shared_tmp_manual = shared;

    // Sending a disable packet to all robots
    for (int n=0; n<3; n++) {
        for (int id=0; id<NB_ROBOT_ELEC; id++) {
            if (shared_tmp_manual.final_control_for_robots[id].is_manually_controled_by_viewer) {
                shared_tmp_manual.final_control_for_robots[id].control.active = false;
                shared_tmp_manual.final_control_for_robots[id].control.ignore = false;
            }
        }
        data << shared_tmp_manual;
        QThread::msleep(25);
    }

    // Restoring the data state
    data << shared;

    for (int id=0; id<NB_ROBOT_ELEC; id++) {
        RhobanSSL::Control &control = shared.final_control_for_robots[id].control;
        if (simulation) {
            if (id <= 7) {
                control.ignore = false;
            }
        } else {
            // XXX: We should not access directly master for this, but use the hardware
            // present flag?
            RhobanSSL::Master *master = dynamic_cast<RhobanSSL::AICommanderReal*>(commander)->getMaster();
            auto masterRobot = master->robots[id];
            control.ignore = !masterRobot.isOk();

            if (id == 3) {
                std::cout << "Age: " << masterRobot.age() << std::endl;
            }
            if (masterRobot.isOk()) {
                std::cout << "Robot #" << id << " is enabled!" << std::endl;
            }
        }
    }
    data << shared;

    mutex.unlock();
}

void API::joystickThreadExec()
{
    static bool fast = false;
    RhobanSSL::Joystick::JoystickEvent event;
    if (joystick != NULL) {
        joystick->open();
    }

    {
        RhobanSSL::Shared_data shared;
        data >> shared;
        auto &final_control = shared.final_control_for_robots[joystickRobot];
        final_control.is_manually_controled_by_viewer = true;
        data << shared;
    }

    float xSpeed = 0;
    float ySpeed = 0;
    float thetaSpeed = 0;
    bool spin = false;
    bool charge = false;
    int kick = false;

    while ((joystickRobot >= 0) && (joystick != NULL)) {
        while (joystick->getEvent(&event)) {
            mutex.lock();
            if (event.type == JS_EVENT_AXIS) {
                if (event.number == 0) {
                    ySpeed = -(fast ? 2.0 : 0.5)*event.getValue();
                }
                if (event.number == 1) {
                    xSpeed = -(fast ? 2.0 : 0.5)*event.getValue();
                }
                if (event.number == 3) {
                    thetaSpeed = -1.5*event.getValue();
                }

                std::cout << "[JOYSTICK] Axis #" << (int)event.number << ": " << event.getValue() << std::endl;;
            }
            if (event.type == JS_EVENT_BUTTON) {
                if (event.number == 4) {
                    spin = event.isPressed();
                }
                if (event.number == 3 && event.isPressed()) {
                    charge = !charge;
                }
                if (event.number == 5) {
                    kick = event.isPressed() ? 1 : 0;
                }
                if (event.number == 7) {
                    kick = event.isPressed() ? 2 : 0;
                }
                if (event.number == 6) {
                    fast = event.getValue();
                }

                std::cout << "[JOYSTICK] Button #" << (int)event.number << ": " << (int)event.isPressed() << std::endl;;
            }

            // Updating control
            RhobanSSL::Shared_data shared;
            data >> shared;
            RhobanSSL::Control &control = shared.final_control_for_robots[joystickRobot].control;
            control.velocity_translation = Vector2d(xSpeed, ySpeed);
            control.velocity_rotation = ContinuousAngle(thetaSpeed);
            control.spin = spin;
            control.charge = charge;
            control.kick = false;
            control.chipKick = false;
            if (kick == 1) control.kick = true;
            if (kick == 2) control.chipKick = true;
            data << shared;

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
            this->joystickThreadExec();
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

    Data_for_viewer data_for_viewer;
    data >> data_for_viewer;

    return QString::fromStdString(
        data_for_viewer.annotations.toJsonString()
    );
}

void API::updateAssignments()
{
    std::map<std::string, std::vector<int>> strategies;

    for (auto &entry : assignments) {
        if (entry.second != "") {
            strategies[entry.second].push_back(entry.first);
        }
    }

    auto manual = ai->getManualManager();
    manual->clear_strategy_assignement();
    double t = ai->getCurrentTime();

    for (auto &entry : strategies) {
        manual->assign_strategy(entry.first, t, entry.second);
    }
}

void API::applyStrategy(int id, QString name)
{
    assignments[id] = name.toStdString();
    updateAssignments();
}

void API::clearAssignments()
{
    assignments.clear();
    updateAssignments();
}

QString API::getStrategies()
{
    Json::Value json(Json::arrayValue);

    auto manual = ai->getManualManager();
    for (auto &strategy : manual->get_available_strategies()) {
        json.append(strategy);
    }

    return js(json);
}

QString API::getManagers()
{
    Json::Value json(Json::arrayValue);

    for (auto &manager : ai->getAvailableManagers()) {
        json.append(manager);
    }

    return js(json);
}

void API::setManager(QString manager)
{
    ai->setManager(manager.toStdString());
}

void API::managerStop()
{
    ai->setManager(Manager::names::manual);
    clearAssignments();
}

void API::managerPlay()
{
    RhobanSSL::Shared_data shared;
    data >> shared;

    for (int id=0; id<NB_ROBOT_ELEC; id++) {
        auto &final_control = shared.final_control_for_robots[id];
        final_control.is_manually_controled_by_viewer = false;
    }

    data << shared;
}
