#pragma once

#include <thread>
#include <vision/AIVisionClient.h>
#include <com/AICommander.h>
#include <joystick/Joystick.h>
#include <json/json.h>
#include <string>
#include <QObject>
#include <Data.h>
#include <Ai.h>

class API : public QObject
{
    Q_OBJECT

public:
    API(
        std::string teamName, bool simulation,
        RhobanSSL::Ai::Team team, 
        RhobanSSL::AICommander *commander,
        const std::string & config_path
    );
    virtual ~API();

    bool simulation;

signals:

public slots:
    bool isSimulation();
    bool isYellow();

    // Vision communication statistics
    QString visionStatus();

    // Status of the referee
    QString refereeStatus();

    // Get the status of all robots
    QString robotsStatus();

    // Gets the status of the ball
    QString ballStatus();

    // Gets the geometry of the field
    QString fieldStatus();

    // Scan for robots
    void scan();

    // Enable/disable a robot
    void enableRobot(int id, bool enabled);

    // Manual control a robot
    void manualControl(int id, bool manual);

    // Active the robot
    void activeRobot(int id, bool active);

    // Commands a robot
    void robotCommand(int id,
        double xSpeed=0, double ySpeed=0, double thetaSpeed=0);

    // Enable or disable the charge for a robot
    void robotCharge(int id, bool charge=false);

    // Run a kick
    void kick(int id, int kick);
    
    // Set spin
    void setSpin(int id, bool spin);

    // Emergency stop
    void emergencyStop();

    // Setting the position of the ball or of robot
    void moveBall(double x, double y);
    void moveRobot(bool yellow, int id, double x, double y, double theta);

    // Joystick
    QString availableJoysticks();
    void openJoystick(int robot, QString name);
    void stopJoystick();

    // Tweak the PID of a robot (currently a hack to send hard-coded PID values)
    void tweakPid(int id);

    // Get the annotations that needs to be drawn on the screen
    QString getAnnotations();

    // Getting the strategies
    QString getStrategies();
    void updateAssignments();
    void applyStrategy(int id, QString name);
    void clearAssignments();

    // Getting managers
    QString getManagers();
    void setManager(QString manager);
    void managerStop();
    void managerPlay();

protected:
    std::string teamName;
    RhobanSSL::AI *ai;
    RhobanSSL::Data data;
    RhobanSSL::Ai::Team team;
    RhobanSSL::AIVisionClient visionClient;
    RhobanSSL::AICommander *commander;

    std::map<int, std::string> assignments;

    std::string ourColor();
    std::string opponentColor();

    std::thread *comThread;
    std::thread *aiThread;

    std::mutex mutex;

    std::thread *joystickThread;
    RhobanSSL::Joystick *joystick;
    int joystickRobot;

    // void comThreadExec();
    void aiThreadExec();
    void joystickThreadExec();
};
