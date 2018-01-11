#include <iostream>
#include <unistd.h>
#include "joystick/Joystick.h"
#include "Kinematic.h"
#include "Master.h"

using namespace RhobanSSL;

int main()
{
    // Joystick
    Joystick joystick;
    Joystick::JoystickEvent event;

    if (!joystick.open()) {
        std::cerr << "Can't open joystick" << std::endl;
        return 1;
    }

    // Openning connection with robot
    Master master("/dev/ttyACM0", 1000000);

    bool charge = false;
    double xSpeed = 0;
    double ySpeed = 0;
    double thetaSpeed = 0;
    Kinematic kinematic;

    master.robots[0].kp = 650;
    master.robots[0].ki = 4;
    master.robots[0].kd = 0;
    master.robots[0].kickPower = 10000;

    while (true) {
        master.send();

        while (joystick.getEvent(&event)) {
            if (event.type == JS_EVENT_BUTTON) {
                if (event.number == 11) { // Kick
                    if (event.isPressed()) {
                        master.robots[0].actions |= ACTION_KICK1;
                    } else {
                        master.robots[0].actions &= ~ACTION_KICK1;
                    }
                } else if (event.number == 15) { // Charge
                    if (event.isPressed()) {
                        charge = !charge;
                        if (charge) master.robots[0].actions |= ACTION_CHARGE;
                        else master.robots[0].actions &= ~ACTION_CHARGE;
                    }
                } else {
                    std::cout << "Button [" << (int)event.number << "] " << event.type << std::endl;
                }
            }
            if (event.type == JS_EVENT_AXIS && event.number < 20) {
                if (event.number == 0) {        // Y
                    ySpeed = event.getValue()*5;
                } else if (event.number == 1) { // X
                    xSpeed = -event.getValue()*5;
                } else if (event.number == 2) { // Rotation
                    thetaSpeed = -event.getValue()*2;
                }
                std::cout << "Axis [" << (int)event.number << "] " <<
                 event.getValue() << std::endl;
            }
        }


        if (master.statuses[0].status & STATUS_OK) {
            master.robots[0].actions |= ACTION_ON;
            float voltage = master.statuses[0].cap_volt/10.0;
            // std::cout << "Robot OK, capacitor: " << master.statuses[0].cap_volt/10.0 << "V" << std::endl;

            std::cout << "X: " << xSpeed << ", Y: " << ySpeed << ", T: " << thetaSpeed << ", Volts: " << voltage << std::endl;
            auto wheels = kinematic.compute(xSpeed, ySpeed, thetaSpeed);
            master.robots[0].wheel1 = wheels.frontLeft;
            master.robots[0].wheel2 = wheels.backLeft;
            master.robots[0].wheel3 = wheels.backRight;
            master.robots[0].wheel4 = wheels.frontRight;
            std::cout << master.robots[0].wheel1 << std::endl;
            std::cout << master.robots[0].wheel2 << std::endl;
            std::cout << "-" << std::endl;
        } else {
            std::cout << "Robot missing!" << std::endl;
        }

        usleep(5000);
    }
}
