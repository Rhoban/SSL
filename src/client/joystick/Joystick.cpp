#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#include "Joystick.h"

namespace rhoban_ssl
{
bool Joystick::JoystickEvent::isPressed()
{
  return value;
}

float Joystick::JoystickEvent::getValue()
{
  return value / (float(1 << 15));
}

Joystick::Joystick(std::string device) : fd(-1), device(device)
{
}

std::string Joystick::getDeviceName()
{
  if (device != "")
  {
    return device;
  }
  else
  {
    char* e = getenv("JOYSTICK");
    std::string pad;
    if (e == NULL)
    {
      pad = JOYSTICK_DEVNAME;
    }
    else
    {
      pad = std::string(e);
    }

    return pad;
  }
}

bool Joystick::open()
{
  std::string pad = getDeviceName();
  fd = ::open(pad.c_str(), O_RDONLY | O_NONBLOCK); /* read write for force feedback? */

  return fd > 0;
}

bool Joystick::getEvent(JoystickEvent* evt)
{
  int bytes = read(fd, evt, sizeof(evt));

  return (bytes == sizeof(*evt));
}

void Joystick::close()
{
  if (fd > 0)
  {
    ::close(fd);
    fd = -1;
  }
}

std::vector<std::string> Joystick::getAvailablePads()
{
  std::vector<std::string> joysticks;

  DIR* devInput = opendir("/dev/input");

  while (struct dirent* entry = readdir(devInput))
  {
    std::string name(entry->d_name);

    if (name[0] == 'j' && name[1] == 's')
    {
      std::string fullName = "/dev/input/";
      fullName += name;
      joysticks.push_back(fullName);
    }
  }

  return joysticks;
}
}  // namespace rhoban_ssl
