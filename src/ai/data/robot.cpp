#include "robot.h"
#include <math/matrix2d.h>
#include <config.h>

namespace rhoban_ssl
{
namespace data
{
Robot::Robot() : is_goalie(false)
{
}

rhoban_geometry::Point Robot::dribblerCenter(double time) const
{
  Vector2d dribbler_center = getMovement().linearPosition(time);

  Matrix2d rotation_matrix = Matrix2d(
      std::cos(getMovement().angularPosition(time).value()), std::sin(getMovement().angularPosition(time).value()),
      -std::sin(getMovement().angularPosition(time).value()), std::cos(getMovement().angularPosition(time).value()));
  dribbler_center += rotation_matrix * Vector2d(0, ai::Config::robot_center_to_dribbler_center);

  return { dribbler_center.getX(), dribbler_center.getY() };
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
