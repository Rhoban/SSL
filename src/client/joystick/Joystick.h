#pragma once

#include <string>
#include <stdio.h>

#define JOYSTICK_DEVNAME "/dev/input/js1"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

namespace RhobanSSL
{
    class Joystick
    {
    public:
        struct JoystickEvent {
            unsigned int time;
            short value;
            unsigned char type;
            unsigned char number;

            bool isPressed();
            float getValue();
        };

        Joystick();
        std::string getDeviceName();
        bool open();
        void close();
        bool getEvent(JoystickEvent *evt);

    protected:
        int fd;
    };
}
