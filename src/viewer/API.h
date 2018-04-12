#pragma once

#include <thread>
#include <vision/AIVisionClient.h>
#include <com/AICommander.h>
#include <json/json.h>
#include <string>
#include <QObject>
#include <Data.h>

class API : public QObject
{
    Q_OBJECT

public:
    // Robot objects
    struct Robot
    {
        int id;
        bool enabled;

        float xSpeed;
        float ySpeed;
        float thetaSpeed;

        bool spin;
        bool charge;
        int kick;
        int kickPower;
    };

    Robot robots[MAX_ROBOTS];

    API(bool simulation, RhobanSSL::AIVisionClient::Team team, RhobanSSL::AICommander *commander);
    virtual ~API();

    bool simulation;

signals:

public slots:
    bool isSimulation();
    bool isYellow();

    // Vision communication statistics
    QString visionStatus();

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

    // Commands a robot
    void robotCommand(int id,
        double xSpeed=0, double ySpeed=0, double thetaSpeed=0,
        int kick=0, bool spin=false, bool charge=false);

    // Enable or disable the charge for a robot
    void robotCharge(int id, bool charge=false);

    // Run a kick
    void kick(int id, int kick);

    // Emergency stop
    void emergencyStop();

    // Setting the position of the ball or of robot
    void moveBall(double x, double y);
    void moveRobot(bool yellow, int id, double x, double y, double theta);

protected:
    RhobanSSL::Data data;
    RhobanSSL::AIVisionClient::Team team;
    RhobanSSL::AIVisionClient visionClient;
    RhobanSSL::AICommander *commander;

    std::string ourColor();
    std::string opponentColor();

    std::thread *comThread;
    std::mutex mutex;

    void comThreadExec();
};
