#include "robot.h"

namespace rhoban_ssl
{
namespace data
{
Robot::Robot() : is_goalie(false)
{
}

bool Robot::infraRed() const
{
  return (electronics.status & STATUS_IR) ? true : false;
}

bool Robot::driverError() const
{
  return (electronics.status & STATUS_DRIVER_ERR) ? true : false;
}

bool Robot::isOk() const
{
  return (electronics.status & STATUS_OK) ? true : false;
}

}  // namespace data
}  // namespace rhoban_ssl
