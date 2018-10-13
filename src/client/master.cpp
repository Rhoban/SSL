#include <iostream>
#include <cmath>
#include <unistd.h>
#include <rhoban_geometry/point.h>
#include "joystick/Joystick.h"
#include "Kinematic.h"
#include "Master.h"

using namespace RhobanSSL;
using namespace rhoban_geometry;

#define ROBOT   0

int main()
{
    // Joystick
    Joystick joystick;
    Joystick::JoystickEvent event;

    if (!joystick.open()) {
        std::cerr << "Can't open joystick" << std::endl;
        // return 1;
    }

    // Openning connection with robot
    Master master("/dev/ttyACM0", 1000000);

    double maxAcceleration = 2;

    bool charge = false;
    Point targetSpeed(0, 0);
    Point speed(0, 0);
    double thetaSpeed = 0;
    Kinematic kinematic;

    struct packet_master robot;
    robot.actions = 0;
    robot.x_speed = 0;
    robot.y_speed = 0;
    robot.t_speed = 0;
    robot.kickPower = 60;

    struct packet_params params;
    params.kp = 10.0;
    params.ki = 0.8;
    params.kd = 0.0;
    // master.addParamPacket(ROBOT, params);
    // master.send();

#if 0
    // master.setParams(400, 3, 0);
#else
    // master.setParams(10, 0.5, 0.0);
#endif

    while (true) {
        while (joystick.getEvent(&event)) {
            if (event.type == JS_EVENT_BUTTON) {
                if (event.number == 11) { // Kick
                    if (event.isPressed()) {
                        robot.actions |= ACTION_KICK1;
                    } else {
                        robot.actions &= ~ACTION_KICK1;
                    }
                } else if (event.number == 7) { // Dribble
                    printf("DRIBBLE!\n");
                    if (event.isPressed()) {
                        robot.actions |= ACTION_DRIBBLE;
                    } else {
                        robot.actions &= ~ACTION_DRIBBLE;
                    }
                } else if (event.number == 15) { // Charge
                    if (event.isPressed()) {
                        charge = !charge;
                        if (charge) robot.actions |= ACTION_CHARGE;
                        else robot.actions &= ~ACTION_CHARGE;
                    }
                } else {
                    std::cout << "Button [" << (int)event.number << "] " << event.type << std::endl;
                }
            }
            if (event.type == JS_EVENT_AXIS && event.number < 20) {
                if (event.number == 0) {        // Y
                    targetSpeed.y = event.getValue()*1.5;
                } else if (event.number == 1) { // X
                    targetSpeed.x = -event.getValue()*1.5;
                } else if (event.number == 3) { // Rotation
                    thetaSpeed = -event.getValue()*2;
                }
                std::cout << "Axis [" << (int)event.number << "] " <<
                 event.getValue() << std::endl;
            }
        }

        if (true) { // fabs(targetSpeed.getLength() - speed.getLength()) < maxAcceleration*0.005) {
            speed = targetSpeed;
        } else {
            speed = speed + (targetSpeed-speed).normalize(maxAcceleration);
        }

        if (master.robots[ROBOT].isOk()) {
            robot.actions |= ACTION_ON;
            float voltage = master.robots[ROBOT].status.cap_volt;
            // std::cout << "Robot OK, capacitor: " << master.statuses[ROBOT].cap_volt/10.0 << "V" << std::endl;

            // std::cout << "X: " << speed.x << ", Y: " << speed.y << ", T: " << thetaSpeed << ", Volts: " << voltage << std::endl;
            robot.x_speed = speed.x*1000;
            robot.y_speed = speed.y*1000;
            robot.t_speed = thetaSpeed*1000;
            // std::cout << "-" << std::endl;
        } else {
            // std::cout << "Robot missing!" << std::endl;
        }

        master.addRobotPacket(ROBOT, robot);
        master.send();
        usleep(20000);
    }
}
