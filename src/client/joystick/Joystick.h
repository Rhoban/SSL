#pragma once

#include <vector>
#include <string>
#include <stdio.h>

#define JOYSTICK_DEVNAME "/dev/input/js0"

#define JS_EVENT_BUTTON 0x01 /* button pressed/released */
#define JS_EVENT_AXIS 0x02   /* joystick moved */
#define JS_EVENT_INIT 0x80   /* initial state of device */

namespace rhoban_ssl
{
class Joystick
{
public:
  struct JoystickEvent
  {
    unsigned int time;
    short value;
    unsigned char type;
    unsigned char number;

    bool isPressed();
    float getValue();
  };

  Joystick(std::string device = "");
  std::string getDeviceName();
  bool open();
  void close();
  bool getEvent(JoystickEvent* evt);

  static std::vector<std::string> getAvailablePads();

protected:
  int fd;

  std::string device;
};
}  // namespace rhoban_ssl
