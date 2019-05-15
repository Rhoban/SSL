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

}  // namespace data
}  // namespace rhoban_ssl
