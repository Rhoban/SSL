#include <iostream>
#include <cmath>
#include <unistd.h>
#include <geometry/Point.hpp>
#include "joystick/Joystick.h"
#include "Kinematic.h"
#include "Master.h"

using namespace RhobanSSL;

#define ROBOT   1

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
    robot.kickPower = 1500;

#if 0
    // master.setParams(400, 3, 0);
#else
    // master.setParams(10, 0.5, 0.0);
#endif

    while (true) {
        while (joystick.getEvent(&event)) {
            if (event.type == JS_EVENT_BUTTON) {
                if (false && event.number == 11) { // Kick
                    if (event.isPressed()) {
                        robot.actions |= ACTION_KICK1;
                    } else {
                        robot.actions &= ~ACTION_KICK1;
                    }
                } else if (false && event.number == 10) { // Dribble
                    if (event.isPressed()) {
                        robot.actions |= ACTION_DRIBBLE;
                    } else {
                        robot.actions &= ~ACTION_DRIBBLE;
                    }
                } else if (false && event.number == 15) { // Charge
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
                    targetSpeed.y = event.getValue()*3;
                } else if (event.number == 1) { // X
                    targetSpeed.x = -event.getValue()*3;
                } else if (event.number == 2) { // Rotation
                    thetaSpeed = -event.getValue()*3;
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
            float voltage = master.robots[ROBOT].status.cap_volt/10.0;
            // std::cout << "Robot OK, capacitor: " << master.statuses[ROBOT].cap_volt/10.0 << "V" << std::endl;

            std::cout << "X: " << speed.x << ", Y: " << speed.y << ", T: " << thetaSpeed << ", Volts: " << voltage << std::endl;
            robot.x_speed = speed.x;
            robot.y_speed = speed.y;
            robot.t_speed = thetaSpeed;
            std::cout << "-" << std::endl;
        } else {
            std::cout << "Robot missing!" << std::endl;
        }

        master.addRobotPacket(ROBOT, robot);
        master.send();
    }
}
